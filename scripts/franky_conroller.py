from franky import Affine, CartesianMotion, Robot, ReferenceType, Gripper
import rclpy
import rclpy.node import Node
import lerobot_teleoperator_violin as violin_mod
from lerobot.teleoperators.config import TeleoperatorConfig
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation

ROBOT_IP = "192.168.137.2"

class FrankyListener(Node):
    def __init__(self, controller: FrankyController):
        super().__init__("franky_listener")
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/eef_pose",
            self.eef_cb,             # callback
            10,                  # QoS depth
        )
        self.gripper_sub = self.create_subscription(
            JointState,
            "/gripper_command_fr3",
            self.gripper_cb,             # callback
            10,                  # QoS depth
        )
        self.controller = controller

    def eef_cb(self, msg: PoseStamped):
        # initialize initial eef pose only if leader arm is connected
        if self.controller.initial_eef_pose is None and self.controller.leader_connected():
            self.controller.initial_eef_pose = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
            self.get_logger().info(
                "Initial EEF pose:  [%s, %s, %s], [%s, %s, %s, %s]", 
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
            )
            self.controller.initial_q_inverse = Rotation.from_quat([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]).inv().tolist()

        # self.get_logger().info(f"heard: linear x={msg.linear.x:.3f}, z={msg.angular.z:.3f}")
        if self.controler.initial_eef_pose is not None and self.controller.leader_connected():
            self.controller.get_current_pose(msg)

    def gripper_cb(self, msg: JointState):
        # self.get_logger().info(f"heard: linear x={msg.linear.x:.3f}, z={msg.angular.z:.3f}")
        pass

class FrankyController:
    def __init__(self):
        # franka
        self.robot = Robot(ROBOT_IP)
        self.robot.recover_from_errors()
        self.robot.relative_dynamics_factor = 0.05 # slow/safe

        # gripper
        self.gripper = Gripper(ROBOT_IP)
        self.gripper_speed = 0.02
        self.gripper_force = 20.0

        # leader
        self.leader = self.leader_init()

        self.initial_eef_pose = None
        self.initial_q_inverse = None
    
    def leader_init(self):
        cfg_cls = None
        for obj in vars(violin_mod).values():
            if isinstance(obj, type) and issubclass(obj, TeleoperatorConfig) and obj is not TeleoperatorConfig:
                cfg_cls = obj
                break
        assert cfg_cls is not None, "Could not find a TeleoperatorConfig in lerobot_teleoperator_violin"
        
        cfg = cfg_cls(port="/dev/ttyUSB1", id="my_awesome_staraiviolin_arm")
        teleop = cfg_cls.__name__.removesuffix("Config")
        teleop_cls = getattr(violin_mod, teleop)
        self.leader = teleop_cls(cfg)

    def move_arm(self, translation: list, rotation: list):
        # check this code for with quat rotation and reference type
        motion = CartesianMotion(Affine(translation, rotation), ReferenceType.Relative)
        # robot.move(motion)

    def move_gripper(self, width: float, speed = 0.02):
        # move the fingers to a specific width
        success = self.gripper.move(width, speed)

    def get_current_pose(self, msg: PoseStamped):
            # calculate the translation and rotation relative to the initial pose
            translation_rel = [
                msg.pose.position.x - self.initial_eef_pose[0],
                msg.pose.position.y - self.initial_eef_pose[1],
                msg.pose.position.z - self.initial_eef_pose[2],
            ]
            q_current = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
            # relative rotation is q_rel = q_current * q_initial_inverse
            rotation_rel = Rotation.from_quat(q_current) * Rotation.from_quat(self.initial_q_inverse)
            self.move_arm(translation_rel, rotation_rel.tolist())

    def leader_connected(self) -> bool:
        # check if leader arm is connected
        try:
            self.leader.connect()
            print("[INFO] Leader arm connected successfully.", flush=True)
            return True
        except Exception as exc:
            print(f"[WARN] Leader arm not connected; publishing zeros. Error: {exc}", flush=True)
            return False


def main():
    rclpy.init()
    controller = FrankyController()
    node = FrankyListener(controller)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()