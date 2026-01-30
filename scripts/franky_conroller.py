from franky import Affine, CartesianMotion, Robot, ReferenceType, Gripper
import rclpy
from rclpy.node import Node
import lerobot_teleoperator_violin as violin_mod
from lerobot.teleoperators.config import TeleoperatorConfig
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
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
        self.controller = controller

    def eef_cb(self, msg: PoseStamped):
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        r,p,y = Rotation.from_quat(q).as_euler("xyz", degrees=False)

        r = r
        p = p
        y = -y
        # print(f"Current orientation x: {r:.2f}, y: {p:.2f}, z: {y:.2f}", flush=True)
        q_new = Rotation.from_euler("xyz", [r, p, y]).as_quat()

        msg.pose.orientation.x = q_new[0]
        msg.pose.orientation.y = q_new[1]
        msg.pose.orientation.z = q_new[2]
        msg.pose.orientation.w = q_new[3]

        if self.controller.franka_init_pose is None:
            self.controller.franka_init_pose = self.controller.robot.current_cartesian_state.pose.end_effector_pose

        if self.controller.current_eef_pose is None and self.controller.leader_connected:
            # initialize initial eef pose only if leader arm is connected
            self.controller.current_eef_pose = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
            # self.get_logger().info(
            #     "Initial EEF pose:  [%s, %s, %s], [%s, %s, %s, %s]",
            #     msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            #     msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
            # )
            self.controller.current_q_inverse = Rotation.from_quat([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]).inv()

        elif self.controller.current_eef_pose is not None and self.controller.leader_connected:
            # update current eef pose to controller
            self.controller.update_current_pose(msg)

    def gripper_cb(self, msg: JointState):
        if self.controller.leader_connected:
            width = msg.position[0]
            self.controller.update_gripper(width)


class FrankyController:
    def __init__(self):
        # franka
        self.robot = Robot(ROBOT_IP)
        self.robot.recover_from_errors()
        self.robot.relative_dynamics_factor = 0.03 # slow/safe

        # gripper
        self.gripper = Gripper(ROBOT_IP)
        self.gripper_speed = 0.05
        self.gripper_force = 40.0
        self.gripper_closed = False

        # leader
        # self.leader = self._leader_init()
        self.leader_connected = True

        # franka 
        self.franka_init_pose = None

        self.current_eef_pose = None
        self.current_q_inverse = None

    def _leader_init(self):
        cfg_cls = None
        for obj in vars(violin_mod).values():
            if isinstance(obj, type) and issubclass(obj, TeleoperatorConfig) and obj is not TeleoperatorConfig:
                cfg_cls = obj
                break
        assert cfg_cls is not None, "Could not find a TeleoperatorConfig in lerobot_teleoperator_violin"

        cfg = cfg_cls(port="/dev/ttyUSB0", id="my_awesome_staraiviolin_arm")
        teleop = cfg_cls.__name__.removesuffix("Config")
        teleop_cls = getattr(violin_mod, teleop)
        self.leader = teleop_cls(cfg)

        try:
            self.leader.connect()
            print("[INFO] Leader arm connected successfully.", flush=True)
            self.leader_connected = True
        except Exception as exc:
            print(f"[WARN] Leader arm not connected; publishing zeros. Error: {exc}", flush=True)

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
        # print(f"[INFO] Close gripper to width: {0.0} m", flush=True)
        pass 

    def update_current_pose(self, msg: PoseStamped):
        # calculate the translation and rotation relative to the initial pose
        dx = msg.pose.position.x - self.current_eef_pose[0]
        dy = msg.pose.position.y - self.current_eef_pose[1]
        dz = msg.pose.position.z - self.current_eef_pose[2]
        translation_rel = np.array([[dx], [-dy], [-dz]], dtype=np.float64)

        q_current = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        # relative rotation is q_rel = q_current * q_initial_inverse
        q_rel = (Rotation.from_quat(q_current) * self.current_q_inverse).as_quat()
        rotation_rel = q_rel.reshape(4, 1).astype(np.float64)

        # update current relative pose
        self.current_eef_pose = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        self.current_q_inverse = Rotation.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]).inv()

        # read current franka eef pose
        affine_cur = self.robot.current_cartesian_state.pose.end_effector_pose
        # print(f"[INFO] Current Franka EEF pose: translation {affine_cur.translation.flatten()}, quaternion {affine_cur.quaternion.flatten()}", flush=True)

        self._move_arm(translation_rel, rotation_rel)

    def update_gripper(self, width: float):
        # print(f"[INFO] Update gripper to width: {width} m", flush=True)
        if width < 0.04 and not self.gripper_closed:
            print("below threshold, closing gripper")
            success = self.gripper.grasp(0.0, 0.05, self.gripper_force, 1.0)
            
            # self._close_gripper(0.0, self.gripper_speed) # close
            self.gripper_closed = True
        elif width >= 0.04:
            self._move_gripper(width, self.gripper_speed)  # open
            self.gripper_closed = False

        # if self.gripper.state.is_grasped() and :
        #     cur_width = self.gripper.state.width
        #     print(f"[INFO] Gripper is closed. Current width: {cur_width} m", flush=True)
        #     self._move_gripper(cur_width-0.003, self.gripper_speed)  # maintain grip

def main():
    rclpy.init()
    controller = FrankyController()
    node = FrankyListener(controller)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()