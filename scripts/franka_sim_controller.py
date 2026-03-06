#!/usr/bin/env python3
"""Launch Isaac Sim, load a USD scene, and run RMPFlow on FR3.

The controller target is driven by ROS 2 `/eef_pose`.
"""

from __future__ import annotations

import argparse
import time
import uuid
from pathlib import Path

ROBOT_PRIM_PATH = "/Root/fr3_ft/fr3"
DEFAULT_USD_PATH = Path(__file__).resolve().parents[1] / "usd" / "franka_ft_scene.usd"


class EefPoseSubscriber:
    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node

        class _Node(Node):
            def __init__(self):
                super().__init__("fr3_eef_pose_subscriber")
                self.latest_position = None
                self.latest_orientation = None
                self.create_subscription(PoseStamped, "/eef_pose", self._eef_cb, 10)

            def _eef_cb(self, msg: PoseStamped):
                self.latest_position = [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ]
                self.latest_orientation = [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True
        else:
            self._owns_rclpy = False

        self.rclpy = rclpy
        self.node = _Node()
        self._warned_no_pose = False

    def spin_once(self) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def get_pose(self):
        if self.node.latest_position is None or self.node.latest_orientation is None:
            return None
        return self.node.latest_position, self.node.latest_orientation

    def close(self) -> None:
        self.node.destroy_node()
        if self._owns_rclpy and self.rclpy.ok():
            self.rclpy.shutdown()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch Isaac Sim and apply RMPFlow to FR3 in a USD scene")
    parser.add_argument("--usd-path", type=str, default=str(DEFAULT_USD_PATH), help="USD scene to open")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim without UI")
    parser.add_argument("--target-prim-path", type=str, default="/World/rmp_target")
    parser.add_argument(
        "--target-position",
        type=float,
        nargs=3,
        default=[0.45, 0.0, 0.45],
        metavar=("X", "Y", "Z"),
        help="Target position used when creating a missing target prim",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    print("[INFO] Isaac Sim started.", flush=True)

    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, XFormPrim
    from isaacsim.core.utils.prims import create_prim, is_prim_path_valid
    from isaacsim.core.utils.stage import is_stage_loading, open_stage
    from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import (
        RMPFlowController,
    )

    usd_path = Path(args.usd_path).expanduser().resolve()
    if not usd_path.exists():
        simulation_app.close()
        raise FileNotFoundError(f"USD file not found: {usd_path}")

    print(f"[INFO] Opening USD: {usd_path}", flush=True)
    open_stage(str(usd_path))
    while is_stage_loading():
        time.sleep(0.05)

    world = World(stage_units_in_meters=1.0)

    if not is_prim_path_valid(ROBOT_PRIM_PATH):
        raise RuntimeError(
            f"Robot prim not found at '{ROBOT_PRIM_PATH}'. "
            "Update ROBOT_PRIM_PATH in this script if your FR3 prim differs."
        )

    if not is_prim_path_valid(args.target_prim_path):
        create_prim(
            args.target_prim_path,
            "Xform",
            translation=args.target_position,
        )
        print(
            f"[INFO] Created target prim at '{args.target_prim_path}' with position {args.target_position}",
            flush=True,
        )

    run_tag = uuid.uuid4().hex[:8]
    fr3 = world.scene.add(
        SingleArticulation(prim_path=ROBOT_PRIM_PATH, name=f"fr3_ctrl_{run_tag}")
    )
    # Isaac Sim versions differ in XFormPrim signature; prefer positional prim path.
    try:
        target = world.scene.add(XFormPrim(args.target_prim_path, name=f"rmp_target_{run_tag}"))
    except TypeError:
        target = world.scene.add(XFormPrim(args.target_prim_path))
    pose_sub = EefPoseSubscriber()

    world.reset()

    controller = RMPFlowController(name="fr3_rmpflow", robot_articulation=fr3)
    articulation_controller = fr3.get_articulation_controller()

    reset_needed = False

    print("[INFO] RMPFlow loop started. Listening to /eef_pose.", flush=True)

    try:
        while simulation_app.is_running():
            world.step(render=True)
            pose_sub.spin_once()

            if world.is_stopped() and not reset_needed:
                reset_needed = True

            if world.is_playing():
                if reset_needed:
                    world.reset()
                    controller.reset()
                    reset_needed = False

                pose = pose_sub.get_pose()
                if pose is None:
                    if not pose_sub._warned_no_pose:
                        print("[WARN] Waiting for /eef_pose before applying RMPFlow actions.", flush=True)
                        pose_sub._warned_no_pose = True
                    continue

                target_position, target_orientation = pose
                # Keep the USD target in sync for visualization/debugging.
                try:
                    target.set_world_pose(position=target_position, orientation=target_orientation)
                except TypeError:
                    target.set_world_pose(target_position, target_orientation)
                actions = controller.forward(
                    target_end_effector_position=target_position,
                    target_end_effector_orientation=target_orientation,
                )
                articulation_controller.apply_action(actions)
    finally:
        pose_sub.close()
        simulation_app.close()

if __name__ == "__main__":
    main()
