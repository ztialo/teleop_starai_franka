from franky import Affine, CartesianMotion, Robot, ReferenceType, Gripper, RelativeDynamicsFactor
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
LEADER2FRANKA_TSCALE_FACTOR = 1  # ratio of franka2desk / leader2desk


def quat_normalize_xyzw(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        raise ValueError("Zero-norm quaternion")
    return q / n


def quat_conj_xyzw(q):
    # for unit quats, inverse == conjugate
    x, y, z, w = q
    return np.array([-x, -y, -z,  w], dtype=float)


def quat_mul_xyzw(q1, q2):
    # Hamilton product, both [x,y,z,w]
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=float)


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
        # self.controller.update_gripper(width)

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
        # self.robot.relative_dynamics_factor = RelativeDynamicsFactor(0.12, 0.12, 0.05)  # slow/safe
        self.robot.relative_dynamics_factor = 0.1
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

        # filter
        self.eef_q_prev = None
        self.gripper_width_prev = None

    def _move_arm(self, translation: list, rotation: list):
        # t_scaled = self._scale_tranlation(translation)
        motion = CartesianMotion(Affine(translation, rotation), ReferenceType.Absolute)
        print(f"[INFO] Move rotation: {rotation.flatten()}", flush=True)
        self.robot.move(motion, asynchronous=True)
        # self.robot.join_motion()

    def _move_gripper(self, width: float, speed):
        # move the fingers to a specific width
        self.gripper.move_async(width, speed)

    def _scale_translation(self, t):
        return t * LEADER2FRANKA_TSCALE_FACTOR

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

        # delta filter
        if self.eef_q_prev is None:
            self.eef_q_prev = self.leader_eef_q_cur_w
        elif not self._valid_pose_change(self.leader_eef_q_cur_w):
            # skip command if changes are too small
            return
        else:
            self.eef_q_prev = self.leader_eef_q_cur_w

        # express leader pose in its EEF frame (i.e., delta from initial EEF)
        leader_pose_eef = self.leader_base2eef_delta(self.leader_eef_q_cur_w)
        if leader_pose_eef is not None:
            # transform eef pose in leader eef frame to franka eef frame
            eef_q_cmd_franka_base = self.franka_eef2base(leader_pose_eef)
            trans = np.array(eef_q_cmd_franka_base[:3]).reshape(3, 1)
            quat = np.array(eef_q_cmd_franka_base[3:]).reshape(4, 1)
            self._move_arm(trans, quat)

    def update_gripper(self, width: float):
        if self.gripper_width_prev is None:
            self.gripper_width_prev = width
        elif np.linalg.norm(width - self.gripper_width_prev) < 0.01:
            return
        else:
            self.gripper_width_prev = width

        if width < 0.04 and not self.gripper_closed:
            print("below threshold, closing gripper")
            self.gripper.grasp_async(0.0, 0.05, self.gripper_force, 0.0025, 0.1)
            self.gripper_closed = True
        elif width >= 0.04:
            self._move_gripper(0.1, self.gripper_speed)  # open
            self.gripper_closed = False

    def _valid_pose_change(self, eef_q_cur):
        pos_delta = np.linalg.norm(eef_q_cur[:3] - self.eef_q_prev[:3])
        q_cur = quat_normalize_xyzw(np.asarray(eef_q_cur[3:7], dtype=float))
        q_prev = quat_normalize_xyzw(np.asarray(self.eef_q_prev[3:7], dtype=float))

        q_err = quat_mul_xyzw(q_cur, quat_conj_xyzw(q_prev))
        w_err = np.clip(abs(q_err[3]), -1.0, 1.0)
        rot_delta = 2.0 * np.arccos(w_err)

        # translation threshold  & rotation threshold 2 degrees
        t_threshold = 0.001  # 1 mm
        q_threshold = np.deg2rad(2.0)  # 2 degress
        if pos_delta > t_threshold and rot_delta > q_threshold:
            return True
        else:
            return False

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


def main():
    rclpy.init()
    controller = FrankyController()
    node = FrankyListener(controller)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
