from pymycobot.mycobot import MyCobot
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool

class MyCobotBridge(Node):
    def __init__(self):
        super().__init__('mycobot_bridge')
        self.mc = MyCobot('/dev/ttyAMA0', 115200)
        
        self.create_subscription(Float64MultiArray, '/mycobot/joint_angles', self.joint_callback, 10)
        self.create_subscription(Float64MultiArray, '/mycobot/coords', self.coord_callback, 10)
        self.create_subscription(Bool, '/mycobot/gripper', self.gripper_callback, 10)

    def joint_callback(self, msg):
        if len(msg.data) == 6:
            self.mc.send_angles(msg.data, 30)

    def coord_callback(self, msg):
        if len(msg.data) == 6:
            self.mc.send_coords(list(msg.data), 30, 0)

    def gripper_callback(self, msg):
        self.mc.switch_gripper(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MyCobotBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()