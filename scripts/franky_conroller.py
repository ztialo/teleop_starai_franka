from franky import Affine, CartesianMotion, Robot, ReferenceType, Gripper
import rclpy
from rclpy.node import Node
import lerobot_teleoperator_violin as violin_mod
from lerobot.teleoperators.config import TeleoperatorConfig
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation
import numpy as np

ROBOT_IP = "192.168.137.2"


class FrankyListener(Node):
    def __init__(self, controller):
        super().__init__("franky_listener")
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/eef_pose",
            self.eef_cb,             # callback
            3,                  # QoS depth
        )
        self.gripper_sub = self.create_subscription(
            JointState,
            "/gripper_command_fr3",
            self.gripper_cb,             # callback
            1,                  # QoS depth
        )
        self.base2eef_tmat_sub = self.create_subscription(
            Float64MultiArray,
            "/eef_tf_matrix",
            self.tf_cb,
            1,
        )
        self.controller = controller

    def eef_cb(self, msg: PoseStamped):
        if self.controller.Tmat_eef2base_leader is not None:
            # update current eef pose to controller if tmat is ready
            self.controller.update_current_pose(msg)

    def gripper_cb(self, msg: JointState):
        width = msg.position[0]
        self.controller.update_gripper(width)

    def tf_cb(self, msg: Float64MultiArray):
        """ transformation matrix from leader's base to initial eef pose"""
        if self.controller.Tmat_eef2base_leader is None:
            data = np.array(msg.data).reshape(4, 4)
            self.controller.Tmat_eef2base_leader = data


class FrankyController:
    def __init__(self):
        # franka
        self.robot = Robot(ROBOT_IP)
        self.robot.recover_from_errors()
        self.robot.relative_dynamics_factor = 0.075  # slow/safe
        self.robot_eef_init_pose = self.robot.current_pose.end_effector_pose
        self.Tmat_eef2base_franka = self.robot_eef_init_pose.matrix

        # gripper
        self.gripper = Gripper(ROBOT_IP)
        self.gripper_speed = 0.075
        self.gripper_force = 40.0
        self.gripper_closed = False

        # leader
        self.leader_eef_q_cur = None
        self.Tmat_eef2base_leader = None

        # check rotation alignment
        self.aligned = False
        self.correction_tmat = None


    def _move_arm(self, translation: list, rotation: list):
        motion = CartesianMotion(Affine(translation, rotation), ReferenceType.Absolute)
        print(f"[INFO] Move rotation: {rotation.flatten()}", flush=True)
        self.robot.move(motion, asynchronous=True)

    def _move_gripper(self, width: float, speed):
        # move the fingers to a specific width
        success = self.gripper.move_async(width, speed)
        # print(f"[INFO] Move gripper to width: {width} m", flush=True)

    def _close_gripper(self, speed = 0.05, force = 20.0, epsilon_outer = 1.0):
        # self.gripper.grasp(0.0, speed, force, epsilon_outer)
        print(f"[INFO] Close gripper to width: {0.0} m", flush=True)

    def update_current_pose(self, msg: PoseStamped):
        self.leader_eef_q_cur_w = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        # express leader pose in its EEF frame (i.e., delta from initial EEF)
        leader_pose_eef = self.leader_base2eef_delta(self.leader_eef_q_cur_w)
        if leader_pose_eef is not None:
            # transform eef pose in leader eef frame to franka eef frame
            # leader_pose_eef_corrected = self._apply_correction(leader_pose_eef)
            # map that delta into Franka base frame for absolute move
            eef_q_cmd_franka_base = self.franka_eef2base(leader_pose_eef)
            trans = np.array(eef_q_cmd_franka_base[:3]).reshape(3, 1)
            quat = np.array(eef_q_cmd_franka_base[3:]).reshape(4, 1)
            self._move_arm(trans, quat)
            

    def _apply_correction(self, pose_eef_vec: list[float]):
        """Apply rotation correction to leader eef pose vector."""
        # build homogeneous from pose vector
        T_pose_eef = np.eye(4)
        T_pose_eef[:3, :3] = Rotation.from_quat(pose_eef_vec[3:]).as_matrix()
        T_pose_eef[:3, 3] = pose_eef_vec[:3]

        # apply rotation correction
        T_corrected = np.eye(4)
        T_corrected[:3, :3] = self.correction_rotmat @ T_pose_eef[:3, :3]
        T_corrected[:3, 3] = T_pose_eef[:3, 3]

        trans_corrected = T_corrected[:3, 3]
        quat_corrected = Rotation.from_matrix(T_corrected[:3, :3]).as_quat()
        return np.concatenate([trans_corrected, quat_corrected]).tolist()

    def franka_eef2base(self, pose_eef_vec: list[float]):
        """Map a pose expressed in the leader EEF frame into Franka base frame (absolute command)."""
        # build homogeneous from pose vector
        T_pose_eef = np.eye(4)
        T_pose_eef[:3, :3] = Rotation.from_quat(pose_eef_vec[3:]).as_matrix()
        T_pose_eef[:3, 3] = pose_eef_vec[:3]

        # Franka init pose gives base <- eef; apply delta in that frame
        T_pose_base = self.Tmat_eef2base_franka @ T_pose_eef

        trans_base = T_pose_base[:3, 3]
        quat_base = Rotation.from_matrix(T_pose_base[:3, :3]).as_quat()
        return np.concatenate([trans_base, quat_base]).tolist()

    def leader_base2eef_delta(self, pose_world: list[float]):
        """Express leader pose (in base frame) inside the leader EEF frame (delta from initial EEF).

        pose_world: [x, y, z, qx, qy, qz, qw]
        returns [x, y, z, qx, qy, qz, qw] in EEF frame, or None if TF is unavailable.
        """
        if self.Tmat_eef2base_leader is None:
            return None

        T_base2eef = np.linalg.inv(self.Tmat_eef2base_leader)  # base -> eef

        # build homogeneous pose matrix from world pose
        T_pose_base = np.eye(4)
        T_pose_base[:3, :3] = Rotation.from_quat(pose_world[3:]).as_matrix()
        T_pose_base[:3, 3] = pose_world[:3]

        # express pose in eef frame
        T_pose_eef = T_base2eef @ T_pose_base
        trans_eef = T_pose_eef[:3, 3]
        quat_eef = Rotation.from_matrix(T_pose_eef[:3, :3]).as_quat()
        return np.concatenate([trans_eef, quat_eef]).tolist()

    def update_gripper(self, width: float):
        # print(f"[INFO] Update gripper to width: {width} m", flush=True)
        if width < 0.04 and not self.gripper_closed:
            print("below threshold, closing gripper")
            success = self.gripper.grasp_async(0.0, 0.05, self.gripper_force, 0.0025, 0.1)

            # self._close_gripper(0.0, self.gripper_speed) # close
            self.gripper_closed = True
        elif width >= 0.04:
            self._move_gripper(0.1, self.gripper_speed)  # open
            self.gripper_closed = False


def main():
    rclpy.init()
    controller = FrankyController()
    node = FrankyListener(controller)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
