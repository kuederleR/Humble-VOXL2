import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import pyttsx3

class ArmListener(Node):

    def __init__(self):
        super().__init__('arm_listener')
        self.subscription = self.create_subscription(
            Bool,
            'arm_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.engine = pyttsx3.init()

    def listener_callback(self, msg):
        if msg.data:
            self.engine.say("Arming")
        else:
            self.engine.say("Disarming")
        self.engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    arm_listener = ArmListener()
    rclpy.spin(arm_listener)
    arm_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()