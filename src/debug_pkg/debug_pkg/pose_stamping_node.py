import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer
from transforms3d.euler import quat2euler
from geometry_msgs.msg import PoseStamped, Point

class PoseStampingNode(Node):
    def __init__(self):
        super().__init__('ego_path_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher_ = self.create_publisher(Path, '/ego_path', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.path = Path()
        self.path.header.frame_id = 'odom'

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('odom', 'chassis', now)
            pose = PoseStamped()
            pose.header.stamp = trans.header.stamp
            pose.header.frame_id = 'odom'
            pose.pose.position = Point(
				x=trans.transform.translation.x,
				y=trans.transform.translation.y,
				z=trans.transform.translation.z
			)
            pose.pose.orientation = trans.transform.rotation
            self.path.poses.append(pose)
            self.publisher_.publish(self.path)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseStampingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
