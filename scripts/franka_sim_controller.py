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
ENABLE_TELEOP_DEBUG = True
TELEOP_DEBUG_EVERY_N_STEPS = 120
SEND_ORIENTATION_TARGET = True
ENABLE_ACTION_DEBUG = True
ACTION_DEBUG_EVERY_N_STEPS = 120
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
MISSING_POSE_DEBUG_EVERY_N_STEPS = 120
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


def _short_repr(value: Any, limit: int = 160) -> str:
    text = repr(value)
    if len(text) <= limit:
        return text
    return f"{text[:limit]}..."


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
        self._teleop_debug_steps = 0
        self._action_debug_steps = 0
        self._printed_initial_pose_snapshot = False
        self._homing_done = False
        self._homing_elapsed_s = 0.0
        self._home_joint_positions_cache: Optional[np.ndarray] = None
        self._home_joint_targets_available = True
        self._stopped = False

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
            f"pose candidates={self._pose_attr_paths})",
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
        self._latest_target_pose = None
        self._home_cmd_position = None
        self._home_cmd_orientation = None
        self._input_ref_position = None
        self._input_ref_orientation = None
        self._warned_waiting_home_pose = False
        self._disable_target_updates = not ENABLE_TARGET_VISUALIZATION
        self._printed_initial_pose_snapshot = False
        self._homing_done = False
        self._homing_elapsed_s = 0.0
        self._home_joint_positions_cache = None
        self._home_joint_targets_available = True

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
        if event_type in (
            int(omni.timeline.TimelineEventType.PLAY),
        ):
            self._reset_needed = True
            self._warned_waiting_for_pose = False
            self._missing_pose_steps = 0
            self._warned_articulation_init = False
            self._latest_target_pose = None
            if event_type == int(omni.timeline.TimelineEventType.PLAY):
                self._home_cmd_position = None
                self._home_cmd_orientation = None
                self._input_ref_position = None
                self._input_ref_orientation = None
                self._printed_initial_pose_snapshot = False
                self._homing_done = False
                self._homing_elapsed_s = 0.0
                self._home_joint_positions_cache = None
                self._home_joint_targets_available = True
            self._warned_waiting_home_pose = False
            self._teleop_debug_steps = 0
            self._action_debug_steps = 0

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

    def _log_teleop_debug(
        self,
        raw_input_pos: np.ndarray,
        raw_input_ori: np.ndarray,
        delta_pos_world: np.ndarray,
        cmd_pos: np.ndarray,
        cmd_ori: np.ndarray,
    ) -> None:
        if not ENABLE_TELEOP_DEBUG:
            return
        self._teleop_debug_steps += 1
        if self._teleop_debug_steps % TELEOP_DEBUG_EVERY_N_STEPS != 0:
            return
        print(
            f"[DEBUG][teleop] mode(trans={TRANSLATION_MAPPING_MODE}, orient={ORIENTATION_MAPPING_MODE})",
            flush=True,
        )
        print(
            f"[DEBUG][teleop] input_ref_pos={_fmt_seq(self._input_ref_position)} "
            f"input_ref_ori_wxyz={_fmt_seq(self._input_ref_orientation)}",
            flush=True,
        )
        print(
            f"[DEBUG][teleop] home_pos={_fmt_seq(self._home_cmd_position)} "
            f"home_ori_wxyz={_fmt_seq(self._home_cmd_orientation)}",
            flush=True,
        )
        print(
            f"[DEBUG][teleop] raw_input_pos={_fmt_seq(raw_input_pos)} "
            f"raw_input_ori_wxyz={_fmt_seq(raw_input_ori)}",
            flush=True,
        )
        print(
            f"[DEBUG][teleop] delta_pos_world={_fmt_seq(delta_pos_world)} cmd_pos={_fmt_seq(cmd_pos)}",
            flush=True,
        )
        print(f"[DEBUG][teleop] cmd_ori_wxyz={_fmt_seq(cmd_ori)}", flush=True)

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

        self._log_teleop_debug(
            raw_input_pos=np.asarray(input_position, dtype=np.float64),
            raw_input_ori=in_ori,
            delta_pos_world=delta_pos_world,
            cmd_pos=cmd_pos,
            cmd_ori=cmd_ori,
        )

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

    def _log_action_debug(self, actions: Any) -> None:
        if not ENABLE_ACTION_DEBUG:
            return
        self._action_debug_steps += 1
        if self._action_debug_steps % ACTION_DEBUG_EVERY_N_STEPS != 0:
            return
        jp = getattr(actions, "joint_positions", None)
        jv = getattr(actions, "joint_velocities", None)
        je = getattr(actions, "joint_efforts", None)
        norm_jp = float(np.linalg.norm(np.asarray(jp, dtype=np.float64))) if jp is not None else 0.0
        norm_jv = float(np.linalg.norm(np.asarray(jv, dtype=np.float64))) if jv is not None else 0.0
        norm_je = float(np.linalg.norm(np.asarray(je, dtype=np.float64))) if je is not None else 0.0
        print(
            f"[DEBUG][action] |jp|={norm_jp:.3f} |jv|={norm_jv:.3f} |je|={norm_je:.3f}",
            flush=True,
        )

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

    def _print_omnigraph_debug_snapshot(self) -> None:
        print(f"[DEBUG] OmniGraph node path: {self._node_path}", flush=True)
        for attr_path in self._position_attr_paths + self._orientation_attr_paths + self._pose_attr_paths:
            raw = _read_og_attr(attr_path)
            print(f"[DEBUG] {attr_path} -> {_short_repr(raw)}", flush=True)
        for base in self._position_attr_paths:
            print(
                f"[DEBUG] {base}:x/y/z -> "
                f"{_short_repr(_read_og_leaf_scalar(base, 'x'))}, "
                f"{_short_repr(_read_og_leaf_scalar(base, 'y'))}, "
                f"{_short_repr(_read_og_leaf_scalar(base, 'z'))}",
                flush=True,
            )
        for base in self._orientation_attr_paths:
            print(
                f"[DEBUG] {base}:x/y/z/w -> "
                f"{_short_repr(_read_og_leaf_scalar(base, 'x'))}, "
                f"{_short_repr(_read_og_leaf_scalar(base, 'y'))}, "
                f"{_short_repr(_read_og_leaf_scalar(base, 'z'))}, "
                f"{_short_repr(_read_og_leaf_scalar(base, 'w'))}",
                flush=True,
            )

    def _on_update(self, event: Any) -> None:
        del event
        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            return
        self._latest_target_pose = self._read_target_pose_from_omnigraph()

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

        pose = self._latest_target_pose
        if pose is None:
            self._missing_pose_steps += 1
            if not self._warned_waiting_for_pose:
                print("[WARN] Waiting for first valid pose from OmniGraph ROS2 subscribe node.", flush=True)
                self._warned_waiting_for_pose = True
            if self._missing_pose_steps % MISSING_POSE_DEBUG_EVERY_N_STEPS == 0:
                print(
                    f"[WARN] Still no valid pose after {self._missing_pose_steps} physics steps. "
                    "Dumping OmniGraph candidate attribute values:",
                    flush=True,
                )
                self._print_omnigraph_debug_snapshot()
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
        self._log_action_debug(actions)
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


def stop() -> None:
    global _RUNTIME
    if _RUNTIME is None:
        return
    _RUNTIME.shutdown()
    _RUNTIME = None
    print("[INFO] Teleop runtime stopped.", flush=True)


if __name__ == "__main__":
    start()
