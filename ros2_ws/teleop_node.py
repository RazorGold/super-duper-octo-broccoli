from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, Bool
import rclpy
from rclpy.node import Node

class MyCobotTeleop(Node):
    def __init__(self):
        super().__init__('mycobot_teleop')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.coord_pub = self.create_publisher(Float64MultiArray, '/mycobot/coords', 10)
        self.gripper_pub = self.create_publisher(Bool, '/mycobot/gripper', 10)

        self.coords = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x, y, z, rx, ry, rz
        self.step_pos = 5.0      # mm step
        self.step_rot = 5.0      # deg step

    def joy_callback(self, msg: Joy):
        # Left stick: x, y
        self.coords[0] += msg.axes[0] * self.step_pos
        self.coords[1] += msg.axes[1] * self.step_pos

        # Right stick: z, rx
        self.coords[2] += msg.axes[4] * self.step_pos
        self.coords[3] += msg.axes[3] * self.step_rot

        # D-pad or buttons (example): gripper control
        if msg.buttons[0] == 1:  # A
            self.gripper_pub.publish(Bool(data=True))  # open
        if msg.buttons[1] == 1:  # B
            self.gripper_pub.publish(Bool(data=False))  # close

        # Send coords
        coord_msg = Float64MultiArray()
        coord_msg.data = self.coords
        self.coord_pub.publish(coord_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyCobotTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()