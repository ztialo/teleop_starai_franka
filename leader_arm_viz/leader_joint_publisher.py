from __future__ import annotations

import ast
import glob
import inspect
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import lerobot_teleoperator_violin as violin_mod
from lerobot.teleoperators.config import TeleoperatorConfig
import numpy as np
import math

# print("violin_mod loaded from:", violin_mod.__file__, flush=True)

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
        self.declare_parameter("leader_port", "/dev/ttyUSB0")
        self.declare_parameter("leader_id", "my_awesome_staraiviolin_arm")
        self.declare_parameter("auto_detect_leader_port", True)
        self.declare_parameter("leader_motor_topic", "/leader/motor_positions")

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.leader_joint_names = self._coerce_joint_names(
            self.get_parameter("leader_joint_names").value
        )
        self.franka_joint_names = self._coerce_joint_names(
            self.get_parameter("franka_joint_names").value
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.leader_port = str(self.get_parameter("leader_port").value)
        self.leader_id = str(self.get_parameter("leader_id").value)
        self.auto_detect_leader_port = bool(self.get_parameter("auto_detect_leader_port").value)
        self.leader_motor_topic = str(self.get_parameter("leader_motor_topic").value)

        # queue depth of 10
        self.leader_pub = self.create_publisher(JointState, "/leader/joint_states", 10)
        self.gripper_pub = self.create_publisher(JointState, "/gripper_command_fr3", 10)
        self.leader_motor_pub = self.create_publisher(JointState, self.leader_motor_topic, 10)

        period = 1.0 / max(self.publish_rate_hz, 1e-6)
        self.timer = self.create_timer(period, self._on_timer)
        self.gripper_timer = self.create_timer(0.02, self._on_gripper_timer)  # gripper updates at 50 Hz
        self._leader_connected = False
        self._warned_no_leader = False
        self.leader = None

        cfg_cls = None
        for obj in vars(violin_mod).values():
            if isinstance(obj, type) and issubclass(obj, TeleoperatorConfig) and obj is not TeleoperatorConfig:
                cfg_cls = obj
                break
        assert cfg_cls is not None, "Could not find a TeleoperatorConfig in lerobot_teleoperator_violin"

        selected_port = self._resolve_leader_port()
        if selected_port is None:
            self.get_logger().warning(
                "Leader arm not connected; publishing fallback pose. "
                "No serial device found (checked /dev/ttyUSB* and /dev/ttyACM*)."
            )
            return

        cfg = cfg_cls(port=selected_port, id=self.leader_id)
        self._sanitize_config_optionals(cfg)
        teleop = cfg_cls.__name__.removesuffix("Config")
        teleop_cls = getattr(violin_mod, teleop)
        self.leader = teleop_cls(cfg)
        try:
            self.leader.connect()
            self._leader_connected = True
            print(f"[INFO] Leader arm connected successfully on {selected_port}.", flush=True)
        except Exception as exc:
            # Retry once after sanitizing possibly missing config fields.
            if self._retry_connect_with_fallback_cfg(cfg_cls, teleop_cls, selected_port):
                return
            print(
                f"[WARN] Leader arm not connected; publishing fallback pose. "
                f"Port: {selected_port}. Error: {exc}",
                flush=True,
            )

    def _sanitize_config_optionals(self, cfg) -> None:
        fallback_map = {
            "baudrate": 1000000,
            "baud_rate": 1000000,
            "timeout": 1.0,
            "read_timeout": 1.0,
            "write_timeout": 1.0,
        }
        for field, fallback in fallback_map.items():
            if hasattr(cfg, field) and getattr(cfg, field) is None:
                setattr(cfg, field, fallback)
                self.get_logger().warning(
                    f"Config field '{field}' was None; defaulting to {fallback}."
                )

    def _build_fallback_cfg_kwargs(self, cfg_cls, selected_port: str) -> dict:
        kwargs = {"port": selected_port, "id": self.leader_id}
        try:
            sig = inspect.signature(cfg_cls)
        except Exception:
            return kwargs

        for name, param in sig.parameters.items():
            if name in kwargs or name == "self":
                continue
            if param.default is inspect.Parameter.empty:
                continue
            if param.default is not None:
                continue
            if "timeout" in name:
                kwargs[name] = 1.0
            elif "baud" in name:
                kwargs[name] = 1000000
        return kwargs

    def _retry_connect_with_fallback_cfg(self, cfg_cls, teleop_cls, selected_port: str) -> bool:
        try:
            cfg = cfg_cls(**self._build_fallback_cfg_kwargs(cfg_cls, selected_port))
            self._sanitize_config_optionals(cfg)
            self.leader = teleop_cls(cfg)
            self.leader.connect()
            self._leader_connected = True
            print(
                f"[INFO] Leader arm connected successfully on retry with fallback config on {selected_port}.",
                flush=True,
            )
            return True
        except Exception:
            return False

    def _resolve_leader_port(self) -> str | None:
        if self.leader_port and glob.glob(self.leader_port):
            return self.leader_port

        if not self.auto_detect_leader_port:
            return None

        candidates = sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyACM*'))
        if not candidates:
            return None

        if self.leader_port and self.leader_port != candidates[0]:
            self.get_logger().warning(
                f"Requested leader_port '{self.leader_port}' not found; using '{candidates[0]}' instead."
            )
        return candidates[0]

    def _read_leader_joint_positions(self) -> list[float]:
        """
        Return joint positions in radians, same order as self.joint_names.
        Replace this with your Lerobot read code.
        """
        if not self._leader_connected:
            if not self._warned_no_leader:
                self.get_logger().warning(
                    "Leader arm disconnected; continuing to publish fallback pose."
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
        position_list = self._read_leader_joint_positions()

        motor_msg = JointState()
        motor_msg.header.stamp = self.get_clock().now().to_msg()
        if self.frame_id:
            motor_msg.header.frame_id = self.frame_id
        motor_msg.name = self.leader_joint_names
        motor_msg.position = [float(v) for v in position_list]
        self.leader_motor_pub.publish(motor_msg)

        msg = JointState()
        msg.header = motor_msg.header
        if self.frame_id:
            msg.header.frame_id = self.frame_id

        msg.name = self.leader_joint_names
        msg.position = self.position_to_radian(position_list)
        self.leader_pub.publish(msg)

    def _on_gripper_timer(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.frame_id:
            msg.header.frame_id = self.frame_id
        msg.name = ["gripper_joint"]
        position_list = self._read_leader_joint_positions()
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
