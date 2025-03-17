import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PoseToPointPublisher(Node):

    def __init__(self):
        super().__init__('pose_to_point_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            PoseStamped,
            '/qvio',
            self.listener_callback,
            qos_profile)
        self.publisher_ = self.create_publisher(Point, '/test/position', qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        point = Point()
        point.x = msg.pose.position.x
        point.y = msg.pose.position.y
        point.z = msg.pose.position.z
        self.publisher_.publish(point)
        self.get_logger().info(f'Publishing Point: x={point.x}, y={point.y}, z={point.z}')

def main(args=None):
    rclpy.init(args=args)
    pose_to_point_publisher = PoseToPointPublisher()
    rclpy.spin(pose_to_point_publisher)
    pose_to_point_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()