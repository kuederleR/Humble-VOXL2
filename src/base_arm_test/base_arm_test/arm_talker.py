import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool

class ArmTalker(Node):
    def __init__(self):
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        super().__init__('arm_talker')
        self.publisher = self.create_publisher(Bool, '/test/arm', qos_profile)
        self.get_logger().info("ArmTalker node has been started. Waiting for input...")

    def wait_for_input_and_publish(self):
        try:
            while rclpy.ok():
                user_input = input("Enter a command to publish (or 'exit' to quit): ")
                if user_input.lower() == 'arm':
                    self.get_logger().info("Publishing arm command from base.")
                    msg = Bool()
                    msg.data = True
                    self.publisher.publish(msg)
                elif user_input.lower() == 'disarm':
                    self.get_logger().info("Publishing disarm command from base.")
                    msg = Bool()
                    msg.data = False
                    self.publisher.publish(msg)
                elif user_input.lower() == 'exit':
                    self.get_logger().info("Exiting...")
                    break

        except KeyboardInterrupt:
            self.get_logger().info("Interrupted by user.")

def main(args=None):
    rclpy.init(args=args)
    arm_talker = ArmTalker()
    arm_talker.wait_for_input_and_publish()
    arm_talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()