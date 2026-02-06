import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from tf_transformations import quaternion_matrix

class EEFPoseReader(Node):
    def __init__(self):
        super().__init__('eef_pose_reader')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.005, self.read_publish_pose) # 20 Hz
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

            self.last_pose = msg
            self.last_transform = transform
            self.eef_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

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
