import sys
from PyQt6.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_system_default

class ROSNode(Node):
    def __init__(self):
        super().__init__('gui_publisher')
        self.publisher_ = self.create_publisher(Bool, '/test/starling_arm', qos_profile_system_default)

    def publish_arm(self, msg):
        self.publisher_.publish(msg)

class MainWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.initUI()

    def initUI(self):
        self.setWindowTitle('ROS Button Press')
        layout = QVBoxLayout()
        self.setMinimumSize(400, 300)
        self.arm_button = QPushButton('Arm', self)
        self.arm_button.clicked.connect(self.on_arm_click)
        layout.addWidget(self.arm_button)

        self.disarm_button = QPushButton('Disarm', self)
        self.disarm_button.clicked.connect(self.on_disarm_click)
        layout.addWidget(self.disarm_button)

        self.setLayout(layout)
        
    def on_disarm_click(self):
        msg = Bool()
        msg.data = False
        self.ros_node.publish_arm(msg)

    def on_arm_click(self):
        msg = Bool()
        msg.data = True
        self.ros_node.publish_arm(msg)

def main(args=None):
    rclpy.init(args=args)
    ros_node = ROSNode()

    app = QApplication(sys.argv)
    main_window = MainWindow(ros_node)
    main_window.show()

    try:
        sys.exit(app.exec())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()