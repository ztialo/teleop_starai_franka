#!/usr/bin/env python3
"""Attach-only FR3 teleoperation controller driven by OmniGraph ROS2 Bridge data.

This script is intended to run *inside an already running Isaac Sim session*.
It does not import or use rclpy.
"""

from __future__ import annotations

import math
import uuid
from typing import Any, Optional, Sequence, Tuple
import numpy as np

# Lazy-loaded Isaac/Omni modules. This keeps import-time failures informative when
# the script is executed outside a running Isaac Sim process.
og = None
omni = None
mg = None
Usd = None
UsdGeom = None
World = None
SingleArticulation = None
XFormPrim = None
create_prim = None
is_prim_path_valid = None


# Robot and target configuration.
ROBOT_PRIM_PATH = "/Root/fr3_ft/fr3"
TARGET_PRIM_PATH = "/World/rmp_target"
TARGET_INITIAL_POSITION = [0.45, 0.0, 0.45]
ENABLE_TARGET_VISUALIZATION = False

# OmniGraph ROS subscribe node configuration.
# Edit these paths if your Action Graph/node names or output attribute names differ.
ACTION_GRAPH_PATH = "/Root/eef_pose_sub"
ROS2_SUBSCRIBE_POSE_NODE_NAME = "ros2_subscribe_eef_pose"
GRIPPER_ACTION_GRAPH_PATH = "/Root/gripper_controller"
ROS2_SUBSCRIBE_GRIPPER_NODE_NAME = "ros2_subscribe_gripper_command"
RMPFLOW_ROBOT_NAME = "FR3"

# Relative teleop mapping:
# commanded_pose = home_pose (+) delta(input_pose relative to first received pose)
USE_RELATIVE_INPUT = True
EE_HOME_REFERENCE_PRIM_PATH = "/Root/fr3_ft/fr3/fr3_hand_tcp"
EE_HOME_REFERENCE_PRIM_FALLBACK_PATHS = []
POSITION_DELTA_SCALE = 3.0
TRANSLATION_MAPPING_MODE = "world_delta"  # "world_delta" (default) or "local_delta"
# Orientation mapping options:
# 1) delta_q = q_current * conj(q_ref), cmd_q = delta_q * q_home
# 2) delta_q = conj(q_ref) * q_current, cmd_q = q_home * delta_q
ORIENTATION_MAPPING_MODE = "current_conjref__delta_home"  # or "conjref_current__home_delta"
SEND_ORIENTATION_TARGET = True
# Incoming quaternion order for 4-element sequence outputs from OmniGraph.
# ROS geometry_msgs uses (x, y, z, w). Isaac Core APIs expect (w, x, y, z).
OG_SEQUENCE_QUAT_ORDER = "xyzw"  # or "wxyz" if your node already outputs scalar-first
# For ROS2 Subscribe Pose/PoseStamped nodes, output names can vary by node type/version.
# Keep these candidates editable and ordered by preference.
OG_POSITION_OUTPUT_ATTR_CANDIDATES = [
    "outputs:position",
    "outputs:pose:position",
]
OG_ORIENTATION_OUTPUT_ATTR_CANDIDATES = [
    "outputs:orientation",
    "outputs:pose:orientation",
]
OG_POSE_OUTPUT_ATTR_CANDIDATES = [
    "outputs:pose",
    "outputs:data",
    "outputs:message",
    "outputs:data:pose",
]
OG_GRIPPER_OUTPUT_ATTR_CANDIDATES = [
    "outputs:position",
    "outputs:message:position",
    "outputs:data:position",
    "outputs:message",
    "outputs:data",
]
FR3_HOME_JOINT_TARGETS = {
    "fr3_joint1": 0.0,
    "fr3_joint2": -0.785,
    "fr3_joint3": 0.0,
    "fr3_joint4": -2.356,
    "fr3_joint5": 0.0,
    "fr3_joint6": 1.571,
    "fr3_joint7": 0.785,
    "fr3_finger_joint1": 0.04,
    "fr3_finger_joint2": 0.04,
}
FR3_HOME_SETTLE_SECONDS = 0.5
GRIPPER_JOINT_NAMES = ("fr3_finger_joint1", "fr3_finger_joint2")
GRIPPER_MIN_POS = 0.0
GRIPPER_MAX_POS = 0.04
GRIPPER_MAX_SPEED = 0.03  # finger joint position units per second
LEFT_FT_JOINT_PRIM_PATH = "/Root/fr3_ft/fr3/fr3_left_ft/fr3_gripper_left_ft"
RIGHT_FT_JOINT_PRIM_PATH = "/Root/fr3_ft/fr3/fr3_right_ft/fr3_gripper_right_ft"
GRIPPER_CONTACT_FORCE_THRESHOLD = 2.0
FT_FORCE_ATTR_CANDIDATES = [
    "state:normalForce",
    "state:force",
    "state:linearForce",
    "physxJoint:normalForce",
    "physxJoint:force",
]


def _ensure_isaac_imports() -> None:
    global og, omni, mg, Usd, UsdGeom, World, SingleArticulation, XFormPrim, create_prim, is_prim_path_valid
    if og is not None:
        return
    try:
        import omni as _omni
        import omni.graph.core as _og
        import isaacsim.robot_motion.motion_generation as _mg
        from pxr import Usd as _Usd
        from pxr import UsdGeom as _UsdGeom
        from isaacsim.core.api import World as _World
        from isaacsim.core.prims import SingleArticulation as _SingleArticulation
        from isaacsim.core.prims import XFormPrim as _XFormPrim
        from isaacsim.core.utils.prims import create_prim as _create_prim
        from isaacsim.core.utils.prims import is_prim_path_valid as _is_prim_path_valid
    except Exception as exc:
        raise RuntimeError(
            "Isaac Sim Python modules are unavailable in this process. "
            "Run this script inside an already running Isaac Sim app (Script Editor/extension), "
            "not as a standalone `./python.sh script.py` process."
        ) from exc

    omni = _omni
    og = _og
    mg = _mg
    Usd = _Usd
    UsdGeom = _UsdGeom
    World = _World
    SingleArticulation = _SingleArticulation
    XFormPrim = _XFormPrim
    create_prim = _create_prim
    is_prim_path_valid = _is_prim_path_valid


class FR3RmpFlowController:
    def __init__(self, name: str, robot_articulation: Any, physics_dt: float = 1.0 / 60.0) -> None:
        cfg = mg.interface_config_loader.load_supported_motion_policy_config(RMPFLOW_ROBOT_NAME, "RMPflow")
        if cfg is None:
            raise RuntimeError(
                f"Failed to load RMPflow config for robot '{RMPFLOW_ROBOT_NAME}'. "
                "Check motion policy support in Isaac Sim."
            )
        self._motion_policy = mg.lula.motion_policies.RmpFlow(**cfg)
        self._articulation_motion_policy = mg.ArticulationMotionPolicy(
            robot_articulation, self._motion_policy, physics_dt
        )
        self._controller = mg.MotionPolicyController(name=name, articulation_motion_policy=self._articulation_motion_policy)
        self._robot_articulation = robot_articulation
        self._set_base_pose_from_articulation()

    def _set_base_pose_from_articulation(self) -> None:
        base_pos, base_ori = self._robot_articulation.get_world_pose()
        self._motion_policy.set_robot_base_pose(robot_position=base_pos, robot_orientation=base_ori)

    def reset(self) -> None:
        self._controller.reset()
        self._set_base_pose_from_articulation()

    def forward(self, target_end_effector_position: np.ndarray, target_end_effector_orientation: np.ndarray):
        return self._controller.forward(
            target_end_effector_position=target_end_effector_position,
            target_end_effector_orientation=target_end_effector_orientation,
        )


def _as_xyz(value: Any) -> Optional[list[float]]:
    if value is None:
        return None
    if isinstance(value, Sequence) and len(value) == 3:
        xyz = [float(value[0]), float(value[1]), float(value[2])]
    elif all(hasattr(value, attr) for attr in ("x", "y", "z")):
        xyz = [float(value.x), float(value.y), float(value.z)]
    else:
        return None
    if not all(math.isfinite(v) for v in xyz):
        return None
    return xyz


def _as_xyzw(value: Any) -> Optional[list[float]]:
    if value is None:
        return None
    if isinstance(value, Sequence) and len(value) == 4:
        xyzw = [float(value[0]), float(value[1]), float(value[2]), float(value[3])]
    elif all(hasattr(value, attr) for attr in ("x", "y", "z", "w")):
        xyzw = [float(value.x), float(value.y), float(value.z), float(value.w)]
    elif hasattr(value, "GetImaginary") and hasattr(value, "GetReal"):
        imag = value.GetImaginary()
        xyzw = [float(imag[0]), float(imag[1]), float(imag[2]), float(value.GetReal())]
    else:
        return None
    if not all(math.isfinite(v) for v in xyzw):
        return None
    return xyzw


def _xyzw_to_wxyz(xyzw: Sequence[float]) -> list[float]:
    return [float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])]


def _quat_conj_wxyz(q: Sequence[float]) -> np.ndarray:
    return np.asarray([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_mul_wxyz(q1: Sequence[float], q2: Sequence[float]) -> np.ndarray:
    w1, x1, y1, z1 = [float(v) for v in q1]
    w2, x2, y2, z2 = [float(v) for v in q2]
    return np.asarray(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def _quat_normalize_wxyz(q: Sequence[float]) -> np.ndarray:
    q_arr = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q_arr)
    if norm <= 0.0 or not np.isfinite(norm):
        return np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q_arr / norm


def _quat_to_rotmat_wxyz(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = [float(v) for v in _quat_normalize_wxyz(q)]
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.asarray(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _as_wxyz(value: Any) -> Optional[list[float]]:
    if value is None:
        return None
    if hasattr(value, "GetImaginary") and hasattr(value, "GetReal"):
        # USD GfQuat stores scalar separately; convert directly to Isaac scalar-first order.
        imag = value.GetImaginary()
        wxyz = [float(value.GetReal()), float(imag[0]), float(imag[1]), float(imag[2])]
    elif all(hasattr(value, attr) for attr in ("x", "y", "z", "w")):
        # Pose/PoseStamped style orientation fields.
        wxyz = [float(value.w), float(value.x), float(value.y), float(value.z)]
    else:
        xyzw = _as_xyzw(value)
        if xyzw is None:
            return None
        if OG_SEQUENCE_QUAT_ORDER == "wxyz":
            wxyz = [float(xyzw[0]), float(xyzw[1]), float(xyzw[2]), float(xyzw[3])]
        else:
            wxyz = _xyzw_to_wxyz(xyzw)
    if not all(math.isfinite(v) for v in wxyz):
        return None
    return wxyz


def _read_og_attr(attr_path: str) -> Any:
    try:
        return og.Controller.get(attr_path)
    except Exception:
        pass
    try:
        attr = og.Controller.attribute(attr_path)
        return og.Controller.get(attr)
    except Exception:
        return None


def _read_og_leaf_scalar(base_attr_path: str, component: str) -> Optional[float]:
    for attr_path in (f"{base_attr_path}:{component}", f"{base_attr_path}.{component}"):
        value = _read_og_attr(attr_path)
        if value is None:
            continue
        try:
            return float(value)
        except Exception:
            continue
    return None


def _fmt_seq(value: Sequence[float] | np.ndarray, decimals: int = 3) -> str:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    return "[" + ", ".join(f"{v:.{decimals}f}" for v in arr) + "]"


def _extract_pose_from_struct(raw_pose: Any) -> Optional[Tuple[list[float], list[float]]]:
    if raw_pose is None:
        return None

    # PoseStamped-like container.
    if hasattr(raw_pose, "pose"):
        pose_obj = raw_pose.pose
        pos = _as_xyz(getattr(pose_obj, "position", None))
        ori = _as_wxyz(getattr(pose_obj, "orientation", None))
        if pos is not None and ori is not None:
            return pos, ori

    # Pose-like container.
    pos = _as_xyz(getattr(raw_pose, "position", None))
    ori = _as_wxyz(getattr(raw_pose, "orientation", None))
    if pos is not None and ori is not None:
        return pos, ori
    return None


def _extract_gripper_width(raw_msg: Any) -> Optional[float]:
    if raw_msg is None:
        return None

    names = None
    positions = None

    if hasattr(raw_msg, "name"):
        names = getattr(raw_msg, "name", None)
    if hasattr(raw_msg, "position"):
        positions = getattr(raw_msg, "position", None)

    if isinstance(raw_msg, dict):
        names = raw_msg.get("name", names)
        positions = raw_msg.get("position", positions)

    if positions is None and isinstance(raw_msg, Sequence) and not isinstance(raw_msg, (str, bytes)):
        if len(raw_msg) > 0:
            try:
                val = float(raw_msg[0])
                return val if math.isfinite(val) else None
            except Exception:
                return None

    if positions is not None and isinstance(positions, Sequence) and len(positions) > 0:
        if names is not None and isinstance(names, Sequence):
            for joint_name in ("gripper_joint", "fr3_finger_joint1", "fr3_finger_joint2"):
                try:
                    idx = list(names).index(joint_name)
                    val = float(positions[idx])
                    return val if math.isfinite(val) else None
                except Exception:
                    continue
        try:
            val = float(positions[0])
            return val if math.isfinite(val) else None
        except Exception:
            return None

    try:
        val = float(raw_msg)
        return val if math.isfinite(val) else None
    except Exception:
        return None


class FrankaTeleopAttachRuntime:
    def __init__(self) -> None:
        node_path = f"{ACTION_GRAPH_PATH}/{ROS2_SUBSCRIBE_POSE_NODE_NAME}"
        self._node_path = node_path
        self._position_attr_paths = [
            f"{node_path}.{attr_name}" for attr_name in OG_POSITION_OUTPUT_ATTR_CANDIDATES
        ]
        self._orientation_attr_paths = [
            f"{node_path}.{attr_name}" for attr_name in OG_ORIENTATION_OUTPUT_ATTR_CANDIDATES
        ]
        self._pose_attr_paths = [f"{node_path}.{attr_name}" for attr_name in OG_POSE_OUTPUT_ATTR_CANDIDATES]
        gripper_node_path = f"{GRIPPER_ACTION_GRAPH_PATH}/{ROS2_SUBSCRIBE_GRIPPER_NODE_NAME}"
        self._gripper_attr_paths = [f"{gripper_node_path}.{attr_name}" for attr_name in OG_GRIPPER_OUTPUT_ATTR_CANDIDATES]

        self._fr3: Optional[Any] = None
        self._target: Optional[Any] = None
        self._controller: Optional[Any] = None
        self._articulation_controller = None
        self._world: Optional[Any] = None
        self._robot_ready = False

        self._physics_sub = None
        self._update_sub = None
        self._timeline_sub = None

        self._reset_needed = True
        self._warned_waiting_for_pose = False
        self._missing_pose_steps = 0
        self._warned_articulation_init = False
        self._latest_target_pose: Optional[Tuple[list[float], list[float]]] = None
        self._home_cmd_position: Optional[np.ndarray] = None
        self._home_cmd_orientation: Optional[np.ndarray] = None
        self._input_ref_position: Optional[np.ndarray] = None
        self._input_ref_orientation: Optional[np.ndarray] = None
        self._warned_waiting_home_pose = False
        self._disable_target_updates = not ENABLE_TARGET_VISUALIZATION
        self._printed_initial_pose_snapshot = False
        self._homing_done = False
        self._homing_elapsed_s = 0.0
        self._home_joint_positions_cache: Optional[np.ndarray] = None
        self._home_joint_targets_available = True
        self._stopped = False
        self._latest_gripper_width: Optional[float] = None
        self._gripper_target_width: Optional[float] = None
        self._gripper_applied_width: Optional[float] = None
        self._gripper_joint_indices: Optional[Tuple[int, int]] = None
        self._warned_gripper_joint_missing = False
        self._ft_selected_attr: dict[str, str] = {}
        self._warned_ft_unavailable = False
        self._gripper_contact_latched = False

    def _reset_cycle_state(self) -> None:
        self._reset_needed = True
        self._warned_waiting_for_pose = False
        self._missing_pose_steps = 0
        self._warned_articulation_init = False
        self._latest_target_pose = None
        self._warned_waiting_home_pose = False
        self._latest_gripper_width = None
        self._gripper_target_width = None
        self._gripper_applied_width = None

    def _reset_relative_reference_state(self) -> None:
        self._home_cmd_position = None
        self._home_cmd_orientation = None
        self._input_ref_position = None
        self._input_ref_orientation = None
        self._printed_initial_pose_snapshot = False
        self._homing_done = False
        self._homing_elapsed_s = 0.0
        self._home_joint_positions_cache = None
        self._home_joint_targets_available = True
        self._gripper_joint_indices = None
        self._warned_gripper_joint_missing = False
        self._ft_selected_attr = {}
        self._warned_ft_unavailable = False
        self._gripper_contact_latched = False

    def start(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No USD stage is currently open in Isaac Sim.")

        if not is_prim_path_valid(ROBOT_PRIM_PATH):
            raise RuntimeError(
                f"Robot prim not found at '{ROBOT_PRIM_PATH}'. "
                "Update ROBOT_PRIM_PATH in this script."
            )

        if ENABLE_TARGET_VISUALIZATION and not is_prim_path_valid(TARGET_PRIM_PATH):
            create_prim(TARGET_PRIM_PATH, "Xform", translation=TARGET_INITIAL_POSITION)
            print(
                f"[INFO] Created target prim at '{TARGET_PRIM_PATH}' with position {TARGET_INITIAL_POSITION}",
                flush=True,
            )

        self._world = World.instance()
        if self._world is None:
            self._world = World(stage_units_in_meters=1.0)

        run_tag = uuid.uuid4().hex[:8]
        self._fr3 = self._world.scene.add(SingleArticulation(prim_path=ROBOT_PRIM_PATH, name=f"fr3_ctrl_{run_tag}"))
        self._controller = FR3RmpFlowController(name=f"fr3_rmpflow_{run_tag}", robot_articulation=self._fr3)

        if ENABLE_TARGET_VISUALIZATION:
            try:
                self._target = self._world.scene.add(XFormPrim(TARGET_PRIM_PATH, name=f"rmp_target_{run_tag}"))
            except TypeError:
                self._target = self._world.scene.add(XFormPrim(TARGET_PRIM_PATH))
        else:
            self._target = None
        timeline = omni.timeline.get_timeline_interface()
        self._timeline_sub = timeline.get_timeline_event_stream().create_subscription_to_pop(
            self._on_timeline_event
        )
        update_stream = omni.kit.app.get_app().get_update_event_stream()
        self._update_sub = update_stream.create_subscription_to_pop(self._on_update)
        self._physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )

        print("[INFO] Attach runtime started.", flush=True)
        print(
            f"[INFO] Reading teleop pose from OmniGraph attrs (position candidates={self._position_attr_paths}, "
            f"orientation candidates={self._orientation_attr_paths}, "
            f"pose candidates={self._pose_attr_paths}, "
            f"gripper candidates={self._gripper_attr_paths})",
            flush=True,
        )
        if not ENABLE_TARGET_VISUALIZATION:
            print("[INFO] Target visualization is disabled. Driving FR3 directly from teleop commands.", flush=True)

    def shutdown(self) -> None:
        self._stopped = True
        self._physics_sub = None
        self._update_sub = None
        self._timeline_sub = None
        self._fr3 = None
        self._target = None
        self._controller = None
        self._articulation_controller = None
        self._world = None
        self._robot_ready = False
        self._disable_target_updates = not ENABLE_TARGET_VISUALIZATION
        self._reset_cycle_state()
        self._reset_relative_reference_state()

    def _stop_runtime_after_timeline_end(self) -> None:
        global _RUNTIME
        if self._stopped:
            return
        self.shutdown()
        if _RUNTIME is self:
            _RUNTIME = None
        print("[INFO] Timeline ended. Teleop runtime stopped. Run start() to run again.", flush=True)

    def _on_timeline_event(self, event: Any) -> None:
        event_type = int(event.type)
        stop_like_types = {int(omni.timeline.TimelineEventType.STOP)}
        pause_event = getattr(omni.timeline.TimelineEventType, "PAUSE", None)
        if pause_event is not None:
            stop_like_types.add(int(pause_event))
        if event_type in stop_like_types:
            self._stop_runtime_after_timeline_end()
            return
        if event_type == int(omni.timeline.TimelineEventType.PLAY):
            self._reset_cycle_state()
            self._reset_relative_reference_state()

    def _try_initialize_robot(self) -> bool:
        if self._fr3 is None or self._controller is None:
            return False
        try:
            self._fr3.initialize()
            self._articulation_controller = self._fr3.get_articulation_controller()
            self._controller.reset()
            self._robot_ready = True
            self._warned_articulation_init = False
            return True
        except Exception as exc:
            # In attach mode, physics/simulation view can be temporarily unavailable
            # around timeline transitions. Keep retrying on future steps.
            self._robot_ready = False
            self._articulation_controller = None
            if not self._warned_articulation_init:
                print(
                    "[WARN] Articulation view not ready yet; will retry initialization on next physics steps. "
                    f"Last error: {exc}",
                    flush=True,
                )
                self._warned_articulation_init = True
            return False

    def _build_home_joint_positions(self) -> Optional[np.ndarray]:
        if self._fr3 is None:
            return None
        if self._home_joint_positions_cache is not None:
            return self._home_joint_positions_cache

        try:
            current = np.asarray(self._fr3.get_joint_positions(), dtype=np.float64).reshape(-1)
        except Exception:
            return None
        if current.size == 0:
            return None

        dof_names = getattr(self._fr3, "dof_names", None)
        if dof_names is None:
            vals = list(FR3_HOME_JOINT_TARGETS.values())
            if len(vals) != int(current.size):
                self._home_joint_targets_available = False
                print("[WARN] Could not resolve articulation dof names; skipping auto-homing.", flush=True)
                return None
            target = np.asarray(vals, dtype=np.float64)
            self._home_joint_positions_cache = target
            return target

        target = current.copy()
        for i, name in enumerate(dof_names):
            if name in FR3_HOME_JOINT_TARGETS:
                target[i] = float(FR3_HOME_JOINT_TARGETS[name])
        self._home_joint_positions_cache = target
        return target

    def _apply_home_joint_positions(self) -> bool:
        if self._fr3 is None:
            return False
        target = self._build_home_joint_positions()
        if target is None:
            return False
        try:
            self._fr3.set_joint_positions(target)
            return True
        except Exception as exc:
            if self._home_joint_targets_available:
                print(f"[WARN] Failed to apply auto-home joint target: {exc}", flush=True)
                self._home_joint_targets_available = False
            return False

    def _run_startup_homing(self, dt: float) -> bool:
        if self._homing_done:
            return True
        if not self._home_joint_targets_available:
            self._homing_done = True
            return True

        if self._homing_elapsed_s <= 0.0:
            print(
                "[INFO] Auto-homing FR3 to startup joint target and waiting "
                f"{FR3_HOME_SETTLE_SECONDS:.1f}s before latching initial EEF pose.",
                flush=True,
            )

        _ = self._apply_home_joint_positions()
        self._homing_elapsed_s += float(max(dt, 0.0))
        if self._homing_elapsed_s < float(FR3_HOME_SETTLE_SECONDS):
            return False

        self._homing_done = True
        self._input_ref_position = None
        self._input_ref_orientation = None
        self._home_cmd_position = None
        self._home_cmd_orientation = None
        self._printed_initial_pose_snapshot = False
        print("[INFO] Auto-homing complete. Relative references will now be latched.", flush=True)
        return True

    def _get_home_pose_from_reference_prim(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None
        try:
            for prim_path in [EE_HOME_REFERENCE_PRIM_PATH] + list(EE_HOME_REFERENCE_PRIM_FALLBACK_PATHS):
                prim = stage.GetPrimAtPath(prim_path)
                if not prim or not prim.IsValid():
                    continue
                world_mtx = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                t = world_mtx.ExtractTranslation()
                q = world_mtx.ExtractRotationQuat()
                pos = np.asarray([float(t[0]), float(t[1]), float(t[2])], dtype=np.float64)
                ori = np.asarray(
                    [float(q.GetReal()), float(q.GetImaginary()[0]), float(q.GetImaginary()[1]), float(q.GetImaginary()[2])],
                    dtype=np.float64,
                )
                if prim_path != EE_HOME_REFERENCE_PRIM_PATH:
                    print(f"[WARN] Using fallback home reference prim '{prim_path}'.", flush=True)
                return pos, _quat_normalize_wxyz(ori)
            return None
        except Exception:
            return None

    def _to_relative_command(
        self, input_position: np.ndarray, input_orientation: np.ndarray
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if (
            self._input_ref_position is None
            or self._input_ref_orientation is None
            or self._home_cmd_position is None
            or self._home_cmd_orientation is None
        ):
            return None

        # Keep quaternion sign consistent to avoid discontinuities around +/-q equivalence.
        in_ori = _quat_normalize_wxyz(input_orientation)
        if float(np.dot(in_ori, self._input_ref_orientation)) < 0.0:
            in_ori = -in_ori

        delta_pos_world = np.asarray(input_position, dtype=np.float64) - self._input_ref_position
        if TRANSLATION_MAPPING_MODE == "local_delta":
            # Optional test mode: express translation in leader-reference local frame,
            # then re-express that local delta in home frame.
            r_ref = _quat_to_rotmat_wxyz(self._input_ref_orientation)
            delta_local = r_ref.T @ delta_pos_world
            r_home = _quat_to_rotmat_wxyz(self._home_cmd_orientation)
            cmd_pos = self._home_cmd_position + float(POSITION_DELTA_SCALE) * (r_home @ delta_local)
        else:
            # Default mode: directly apply world-frame delta from first input.
            cmd_pos = self._home_cmd_position + float(POSITION_DELTA_SCALE) * delta_pos_world

        if ORIENTATION_MAPPING_MODE == "conjref_current__home_delta":
            delta_ori = _quat_mul_wxyz(_quat_conj_wxyz(self._input_ref_orientation), in_ori)
            cmd_ori = _quat_normalize_wxyz(_quat_mul_wxyz(self._home_cmd_orientation, delta_ori))
        else:
            delta_ori = _quat_mul_wxyz(in_ori, _quat_conj_wxyz(self._input_ref_orientation))
            cmd_ori = _quat_normalize_wxyz(_quat_mul_wxyz(delta_ori, self._home_cmd_orientation))

        return cmd_pos, cmd_ori

    def _ensure_initial_relative_references(
        self, input_position: np.ndarray, input_orientation: np.ndarray
    ) -> bool:
        if (
            self._input_ref_position is None
            or self._input_ref_orientation is None
            or self._home_cmd_position is None
            or self._home_cmd_orientation is None
        ):
            # Latch command and FR3 EEF references atomically in the same step.
            home_pose = self._get_home_pose_from_reference_prim()
            if home_pose is None:
                if not self._warned_waiting_home_pose:
                    print(
                        f"[WARN] Waiting for FR3 EEF world pose from '{EE_HOME_REFERENCE_PRIM_PATH}' "
                        "before applying relative teleop.",
                        flush=True,
                    )
                    self._warned_waiting_home_pose = True
                return False
            pos, ori = home_pose
            self._input_ref_position = np.asarray(input_position, dtype=np.float64)
            self._input_ref_orientation = _quat_normalize_wxyz(input_orientation)
            self._home_cmd_position = np.asarray(pos, dtype=np.float64)
            self._home_cmd_orientation = _quat_normalize_wxyz(ori)
            self._warned_waiting_home_pose = False

        if not self._printed_initial_pose_snapshot:
            print(
                "[INFO] Initial pose references latched: "
                f"initial_command_pos={_fmt_seq(self._input_ref_position)}, "
                f"initial_command_ori_wxyz={_fmt_seq(self._input_ref_orientation)}, "
                f"initial_fr3_eef_pos={_fmt_seq(self._home_cmd_position)}, "
                f"initial_fr3_eef_ori_wxyz={_fmt_seq(self._home_cmd_orientation)}",
                flush=True,
            )
            self._printed_initial_pose_snapshot = True

        return True

    def _read_target_pose_from_omnigraph(self) -> Optional[Tuple[list[float], list[float]]]:
        # OmniGraph data read happens here each step from the ROS2 Subscribe Pose node outputs.
        raw_position = None
        for attr_path in self._position_attr_paths:
            raw_position = _read_og_attr(attr_path)
            if raw_position is not None:
                break

        raw_orientation = None
        for attr_path in self._orientation_attr_paths:
            raw_orientation = _read_og_attr(attr_path)
            if raw_orientation is not None:
                break

        position = _as_xyz(raw_position)
        orientation = _as_wxyz(raw_orientation)
        if position is not None and orientation is not None:
            return position, orientation

        # Leaf fallback for structured vector/quaternion outputs that may return None at parent path.
        for pos_base in self._position_attr_paths:
            px = _read_og_leaf_scalar(pos_base, "x")
            py = _read_og_leaf_scalar(pos_base, "y")
            pz = _read_og_leaf_scalar(pos_base, "z")
            if None not in (px, py, pz):
                position = [float(px), float(py), float(pz)]
                break
        else:
            position = None

        for ori_base in self._orientation_attr_paths:
            ox = _read_og_leaf_scalar(ori_base, "x")
            oy = _read_og_leaf_scalar(ori_base, "y")
            oz = _read_og_leaf_scalar(ori_base, "z")
            ow = _read_og_leaf_scalar(ori_base, "w")
            if None not in (ox, oy, oz, ow):
                orientation = [float(ow), float(ox), float(oy), float(oz)]
                break
        else:
            orientation = None

        if position is not None and orientation is not None:
            return position, orientation

        for attr_path in self._pose_attr_paths:
            raw_pose = _read_og_attr(attr_path)
            pose = _extract_pose_from_struct(raw_pose)
            if pose is not None:
                return pose
        return None

    def _read_gripper_width_from_omnigraph(self) -> Optional[float]:
        for attr_path in self._gripper_attr_paths:
            raw_val = _read_og_attr(attr_path)
            width = _extract_gripper_width(raw_val)
            if width is not None:
                return width
        return None

    def _get_gripper_joint_indices(self) -> Optional[Tuple[int, int]]:
        if self._gripper_joint_indices is not None:
            return self._gripper_joint_indices
        if self._fr3 is None:
            return None
        dof_names = getattr(self._fr3, "dof_names", None)
        if not isinstance(dof_names, Sequence):
            return None
        try:
            idx1 = list(dof_names).index(GRIPPER_JOINT_NAMES[0])
            idx2 = list(dof_names).index(GRIPPER_JOINT_NAMES[1])
            self._gripper_joint_indices = (int(idx1), int(idx2))
            return self._gripper_joint_indices
        except Exception:
            if not self._warned_gripper_joint_missing:
                print(
                    f"[WARN] Gripper joints {GRIPPER_JOINT_NAMES} not found in articulation DOFs.",
                    flush=True,
                )
                self._warned_gripper_joint_missing = True
            return None

    def _apply_gripper_width(self, width: float) -> None:
        if self._fr3 is None:
            return
        indices = self._get_gripper_joint_indices()
        if indices is None:
            return
        target = float(np.clip(width, GRIPPER_MIN_POS, GRIPPER_MAX_POS))
        try:
            joint_positions = np.asarray(self._fr3.get_joint_positions(), dtype=np.float64).reshape(-1)
            if joint_positions.size <= max(indices):
                return
            joint_positions[indices[0]] = target
            joint_positions[indices[1]] = target
            self._fr3.set_joint_positions(joint_positions)
        except Exception as exc:
            print(f"[WARN] Failed to apply gripper command {target:.3f}: {exc}", flush=True)

    @staticmethod
    def _force_value_to_scalar(value: Any) -> Optional[float]:
        if value is None:
            return None
        try:
            scalar = float(value)
            return abs(scalar) if math.isfinite(scalar) else None
        except Exception:
            pass
        if all(hasattr(value, a) for a in ("x", "y", "z")):
            try:
                vec = np.asarray([float(value.x), float(value.y), float(value.z)], dtype=np.float64)
                n = float(np.linalg.norm(vec))
                return n if math.isfinite(n) else None
            except Exception:
                return None
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) >= 1:
            try:
                arr = np.asarray([float(v) for v in value], dtype=np.float64).reshape(-1)
                n = float(np.linalg.norm(arr))
                return n if math.isfinite(n) else None
            except Exception:
                return None
        return None

    def _read_ft_force_scalar(self, prim_path: str) -> Optional[float]:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return None

        selected = self._ft_selected_attr.get(prim_path)
        attr_names = [selected] if selected else []
        attr_names.extend([name for name in FT_FORCE_ATTR_CANDIDATES if name != selected])
        for attr_name in attr_names:
            if not attr_name:
                continue
            try:
                attr = prim.GetAttribute(attr_name)
                if not attr or not attr.IsValid():
                    continue
                force_val = self._force_value_to_scalar(attr.Get())
                if force_val is None:
                    continue
                self._ft_selected_attr[prim_path] = attr_name
                return force_val
            except Exception:
                continue
        return None

    def _read_gripper_contact_force(self) -> Optional[float]:
        left = self._read_ft_force_scalar(LEFT_FT_JOINT_PRIM_PATH)
        right = self._read_ft_force_scalar(RIGHT_FT_JOINT_PRIM_PATH)
        vals = [v for v in (left, right) if v is not None]
        if not vals:
            if not self._warned_ft_unavailable:
                print(
                    "[WARN] Could not read fingertip FT force values from configured prim paths.",
                    flush=True,
                )
                self._warned_ft_unavailable = True
            return None
        return max(vals)

    def _read_current_gripper_width(self) -> Optional[float]:
        if self._fr3 is None:
            return None
        indices = self._get_gripper_joint_indices()
        if indices is None:
            return None
        try:
            joint_positions = np.asarray(self._fr3.get_joint_positions(), dtype=np.float64).reshape(-1)
            if joint_positions.size <= max(indices):
                return None
            return float(0.5 * (joint_positions[indices[0]] + joint_positions[indices[1]]))
        except Exception:
            return None

    def _update_gripper_smooth(self, dt: float) -> None:
        if self._latest_gripper_width is not None:
            self._gripper_target_width = float(np.clip(self._latest_gripper_width, GRIPPER_MIN_POS, GRIPPER_MAX_POS))
        if self._gripper_target_width is None:
            return

        if self._gripper_applied_width is None:
            current_width = self._read_current_gripper_width()
            self._gripper_applied_width = (
                current_width if current_width is not None else self._gripper_target_width
            )

        contact_force = self._read_gripper_contact_force()
        closing_requested = self._gripper_target_width < (self._gripper_applied_width - 1e-6)
        opening_requested = self._gripper_target_width > (self._gripper_applied_width + 1e-6)

        if opening_requested:
            self._gripper_contact_latched = False
        elif (
            closing_requested
            and contact_force is not None
            and contact_force >= float(GRIPPER_CONTACT_FORCE_THRESHOLD)
        ):
            self._gripper_contact_latched = True

        if self._gripper_contact_latched and closing_requested:
            self._gripper_target_width = self._gripper_applied_width

        max_step = float(GRIPPER_MAX_SPEED) * float(max(dt, 1e-6))
        error = self._gripper_target_width - self._gripper_applied_width
        step = float(np.clip(error, -max_step, max_step))
        self._gripper_applied_width += step
        self._apply_gripper_width(self._gripper_applied_width)

    def _on_update(self, event: Any) -> None:
        del event
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            return
        self._latest_target_pose = self._read_target_pose_from_omnigraph()
        gripper_width = self._read_gripper_width_from_omnigraph()
        if gripper_width is not None:
            self._latest_gripper_width = gripper_width

    def _set_target_world_pose(self, position: Sequence[float], orientation_wxyz: Sequence[float]) -> None:
        if self._disable_target_updates or self._target is None:
            return
        # Isaac API variants:
        # - set_world_pose(position=..., orientation=...)
        # - set_world_poses(positions=..., orientations=...) (batched)
        try:
            self._target.set_world_pose(position=position, orientation=orientation_wxyz)
            return
        except Exception:
            pass
        try:
            self._target.set_world_pose(position, orientation_wxyz)
            return
        except Exception:
            pass
        try:
            positions = np.asarray([position], dtype=np.float32)
            orientations = np.asarray([orientation_wxyz], dtype=np.float32)
            self._target.set_world_poses(positions=positions, orientations=orientations)
            return
        except Exception:
            pass
        try:
            positions = np.asarray([position], dtype=np.float32)
            orientations = np.asarray([orientation_wxyz], dtype=np.float32)
            self._target.set_world_poses(positions, orientations)
        except Exception as exc:
            print(f"[WARN] Failed to update target pose visualization: {exc}", flush=True)
            self._disable_target_updates = True
            self._target = None

    def _on_physics_step(self, dt: float) -> None:
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            return

        if self._controller is None or self._fr3 is None:
            return

        if self._reset_needed:
            # Never call world.reset() inside physics callback (can recurse/crash).
            # Reinitialize articulation/controller state in-place when timeline restarts.
            if not self._try_initialize_robot():
                return
            self._reset_needed = False

        if not self._robot_ready or self._articulation_controller is None:
            self._try_initialize_robot()
            return

        if not self._run_startup_homing(dt):
            return

        self._update_gripper_smooth(dt)

        pose = self._latest_target_pose
        if pose is None:
            self._missing_pose_steps += 1
            if not self._warned_waiting_for_pose:
                print("[WARN] Waiting for first valid pose from OmniGraph ROS2 subscribe node.", flush=True)
                self._warned_waiting_for_pose = True
            return
        self._missing_pose_steps = 0

        target_position, target_orientation = pose
        target_position_np = np.asarray(target_position, dtype=np.float64)
        target_orientation_np = _quat_normalize_wxyz(np.asarray(target_orientation, dtype=np.float64))
        if USE_RELATIVE_INPUT:
            if not self._ensure_initial_relative_references(target_position_np, target_orientation_np):
                return
            relative_cmd = self._to_relative_command(
                target_position_np, target_orientation_np
            )
            if relative_cmd is None:
                return
            target_position_np, target_orientation_np = relative_cmd

        # Keep the target Xform synced to incoming pose for debugging in the stage.
        self._set_target_world_pose(target_position_np, target_orientation_np)

        target_ori_arg = target_orientation_np if SEND_ORIENTATION_TARGET else None
        actions = self._controller.forward(
            target_end_effector_position=target_position_np,
            target_end_effector_orientation=target_ori_arg,
        )
        self._articulation_controller.apply_action(actions)


_RUNTIME: Optional[FrankaTeleopAttachRuntime] = None


def start() -> None:
    global _RUNTIME
    _ensure_isaac_imports()
    if _RUNTIME is not None:
        print("[WARN] Teleop runtime already started.", flush=True)
        return
    _RUNTIME = FrankaTeleopAttachRuntime()
    _RUNTIME.start()


if __name__ == "__main__":
    start()
