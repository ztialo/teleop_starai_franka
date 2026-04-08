import rclpy
import numpy as np
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from tf_transformations import quaternion_matrix

VALID_POS_DELTA_THRESHOLD = 0.02 # meter
VALID_ROT_DELTA_THRESHOLD = 0.05 # radian (=3 degrees)
MIN_POS_DELTA_THRESHOLD = 0.003 # 3 mm
MIN_ROT_DELTA_THRESHOLD = np.deg2rad(0.5)

""" HELPER FUNCTIONS """


def _position_to_array(pose_stamped: PoseStamped) -> np.ndarray:
    return np.array(
        [
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
        ],
        dtype=np.float64,
    )


def _quaternion_to_array(pose_stamped: PoseStamped) -> np.ndarray:
    quat = np.array(
        [
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w,
        ],
        dtype=np.float64,
    )
    norm = np.linalg.norm(quat)
    if norm == 0.0:
        raise ValueError("Received zero-norm quaternion in pose filter")
    return quat / norm


""" HELPER FUNCTIONS ENDS """


class EEFPoseReader(Node):
    def __init__(self):
        super().__init__('eef_pose_reader')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.read_publish_pose) # 10 Hz
        self.eef_pub = self.create_publisher(PoseStamped, '/eef_pose', 10)
        self.tf_mat_pub = self.create_publisher(Float64MultiArray, '/eef_tf_matrix', 10)
        self.tf_matrix_timer = self.create_timer(1.0, self.publish_tf_matrix)  # slow timer
        self.last_pose: PoseStamped | None = None
        self.last_transform = None

    def read_publish_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'link6',
                rclpy.time.Time()
            )

            t = transform.transform.translation
            q = transform.transform.rotation
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.pose.position.x = t.x
            msg.pose.position.y = t.y
            msg.pose.position.z = t.z
            msg.pose.orientation = q

            result = self.pose_filter(msg, self.last_pose)

            # only publish if pose data passes filter
            if result is not None:
                self.last_pose = result
                self.last_transform = transform
                self.eef_pub.publish(result)

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def pose_filter(self, new_pose: PoseStamped, last_pose: PoseStamped | None):
        """Add filter for noise and deadband"""
        if last_pose is None:
            return new_pose

        # Calculate the Euclidean distance change
        new_pos = _position_to_array(new_pose)
        last_pos = _position_to_array(last_pose)
        dp = new_pos - last_pos
        dist = np.linalg.norm(dp)

        # Calculate total angle change
        new_quat = _quaternion_to_array(new_pose)
        last_quat = _quaternion_to_array(last_pose)
        dot = np.dot(new_quat, last_quat)
        dot = np.clip(dot, -1.0, 1.0)  # safety
        angle = 2 * np.arccos(abs(dot))  # radian

        # filter noise and huge spikes
        if dist > VALID_POS_DELTA_THRESHOLD:
            return None
        if angle > VALID_ROT_DELTA_THRESHOLD:
            return None

        # deadband filter, suppress command with too small of a change
        if dist < MIN_POS_DELTA_THRESHOLD and angle < MIN_ROT_DELTA_THRESHOLD:
            return None

        return new_pose

    def publish_tf_matrix(self):
        """Publish the 4x4 transform matrix at a slower rate for consumers that need it once in a while."""
        try:
            # transform from source to target
            transform = self.tf_buffer.lookup_transform(
                'base_link', # target
                'link6',  # source 
                rclpy.time.Time()
            )
        except Exception:
            transform = self.last_transform

        if transform is None:
            return

        t = transform.transform.translation
        q = transform.transform.rotation

        T = quaternion_matrix([q.x, q.y, q.z, q.w])  # 4x4 numpy array
        T[0, 3], T[1, 3], T[2, 3] = t.x, t.y, t.z

        mat_msg = Float64MultiArray()
        mat_msg.data = T.flatten().tolist()
        self.tf_mat_pub.publish(mat_msg)


def main():
    rclpy.init()
    node = EEFPoseReader()
    rclpy.spin(node)
    rclpy.shutdown()
