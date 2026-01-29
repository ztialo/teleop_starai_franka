import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped

class EEFPoseReader(Node):
    def __init__(self):
        super().__init__('eef_pose_reader')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.01, self.read_publish_pose) # 100 Hz
        self.eef_pub = self.create_publisher(PoseStamped, '/eef_pose', 10)
        self.last_pose: PoseStamped | None = None

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
            self.eef_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")


def main():
    rclpy.init()
    node = EEFPoseReader()
    rclpy.spin(node)
    rclpy.shutdown()
