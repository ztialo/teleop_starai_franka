from __future__ import annotations

import ast
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# import lerobot_teleoperator_violin as violin_mod
# from lerobot.teleoperators.config import TeleoperatorConfig
import numpy as np
import math


leader_limits = {
    "joint1": (-100.0, 100.0),
    "joint2": (-84.0, 100.0),
    "joint3": (-100.0, 100.0),
    "joint4": (-100.0, 100.0),
    "joint5": (-100.0, 100.0),
    "joint6": (-98.0, 100.0),
    "joint7_left": (0.0, 100.0),
}

JOINT146_POS2RAD_SCALAR = 0.0227
JOINT235_POS2RAD_SCALAR = 0.0157
JOINT7_POS2RAD_SCALAR = 0.00025
JOINT7_POS2RAD_OFFSET = 0.025

class LeaderJointPublisher(Node):
    def __init__(self):
        super().__init__("leader_joint_publisher")

        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("leader_joint_names", ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7_left",])
        self.declare_parameter("franka_joint_names", ["fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7",])   
        self.declare_parameter("frame_id", "")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.leader_joint_names = self._coerce_joint_names(
            self.get_parameter("leader_joint_names").value
        )
        self.franka_joint_names = self._coerce_joint_names(
            self.get_parameter("franka_joint_names").value
        )
        self.frame_id = str(self.get_parameter("frame_id").value)

        # queue depth of 10
        self.leader_pub = self.create_publisher(JointState, "/leader/joint_states", 10)
        self.franka_pub = self.create_publisher(JointState, "/joint_command_fr3", 10)
        self.gripper_pub = self.create_publisher(JointState, "/gripper_command_fr3", 10)

        period = 1.0 / max(self.publish_rate_hz, 1e-6)
        self.timer = self.create_timer(period, self._on_timer)
        self._leader_connected = False
        self._warned_no_leader = False

        # cfg_cls = None
        # for obj in vars(violin_mod).values():
        #     if isinstance(obj, type) and issubclass(obj, TeleoperatorConfig) and obj is not TeleoperatorConfig:
        #         cfg_cls = obj
        #         break
        # assert cfg_cls is not None, "Could not find a TeleoperatorConfig in lerobot_teleoperator_violin"
        
        # cfg = cfg_cls(port="/dev/ttyUSB1", id="my_awesome_staraiviolin_arm")
        # teleop = cfg_cls.__name__.removesuffix("Config")
        # teleop_cls = getattr(violin_mod, teleop)
        # self.leader = teleop_cls(cfg)
        # try:
        #     self.leader.connect()
        #     self._leader_connected = True
        #     print("[INFO] Leader arm connected successfully.", flush=True)
        # except Exception as exc:
        #     print(f"[WARN] Leader arm not connected; publishing zeros. Error: {exc}", flush=True)

    def _read_leader_joint_positions(self) -> list[float]:
        """
        Return joint positions in radians, same order as self.joint_names.
        Replace this with your Lerobot read code.
        """
        if not self._leader_connected:
            if not self._warned_no_leader:
                self.get_logger().warning(
                    "Leader arm disconnected; continuing to publish zero joint positions."
                )
                self._warned_no_leader = True
            return [0.0, 0.157, 0.0628, 0.0, 1.413, -1.2939, -0.025]  # home position in radians

        observe = self.leader.get_action()
        keys = sorted(observe.keys())
        joint_vals = []
        for k in keys:
            v = observe[k]
            if hasattr(v, "detach"): v = v.detach().cpu().numpy()
            v = np.array(v).reshape(-1)[0]
            joint_vals.append(float(v))
        return joint_vals

    def position_to_radian(self, positions) -> float:
        radians = []
        for i in range(len(positions)):
            if i == 0 or i == 3 or i == 5:
                radians.append(positions[i] * JOINT146_POS2RAD_SCALAR)
            elif i == 1 or i == 2 or i == 4:
                radians.append(positions[i] * JOINT235_POS2RAD_SCALAR)
            elif i == 6:
                radians.append((positions[i] * JOINT7_POS2RAD_SCALAR) - JOINT7_POS2RAD_OFFSET)
        return radians

    def _coerce_joint_names(self, value) -> list[str]:
        if isinstance(value, list):
            return [str(item) for item in value]
        if isinstance(value, str):
            try:
                parsed = ast.literal_eval(value)
            except (SyntaxError, ValueError):
                return [value]
            if isinstance(parsed, list):
                return [str(item) for item in parsed]
            return [str(parsed)]
        return [str(value)]

    def _on_timer(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.frame_id:
            msg.header.frame_id = self.frame_id

        msg.name = self.leader_joint_names
        position_list = self._read_leader_joint_positions()
        msg.position = self.position_to_radian(position_list)
        self.leader_pub.publish(msg)
        
        # Consider we are locking the 3rd joint!!!! 
        # reusing the sample joint position values to control franka arm for now
        msg.name = self.franka_joint_names
        msg.position = self.position_to_radian(position_list)
        self.franka_pub.publish(msg)

        msg.name = ["gripper_joint"]
        # publish raw position (not radians) for the gripper joint
        msg.position = [float(position_list[6])/1000.0]
        self.gripper_pub.publish(msg)


def main():
    rclpy.init()
    node = LeaderJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
