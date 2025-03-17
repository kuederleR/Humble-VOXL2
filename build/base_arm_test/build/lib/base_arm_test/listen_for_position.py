import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_system_default

class PositionListener(Node):

    def __init__(self):
        super().__init__('position_listener')
        self.subscription = self.create_subscription(
            Point,
            '/test/position',
            self.listener_callback,
            qos_profile_system_default)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received point: x={msg.x}, y={msg.y}, z={msg.z}')

def main(args=None):
    rclpy.init(args=args)
    position_listener = PositionListener()
    rclpy.spin(position_listener)
    position_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()