import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pymycobot.mycobot import MyCobot

class MyCobotTeleop(Node):
    def __init__(self):
        super().__init__('mycobot_teleop')
        # Parameters for serial port, etc.
        port = self.declare_parameter('port', '/dev/ttyAMA0').get_parameter_value().string_value
        baud = self.declare_parameter('baud', 115200).get_parameter_value().integer_value

        # Connect to myCobot hardware
        self.mc = MyCobot(port, baud)
        # Get current coords as starting target (x, y, z in mm, rx, ry, rz in deg)
        self.target_coords = self.mc.get_coords()  # e.g. [x,y,z,rx,ry,rz]

        # Teleop tuning parameters
        self.declare_parameter('step_time', 0.1)  # control loop period in seconds
        self.step_time = self.get_parameter('step_time').get_parameter_value().double_value
        self.max_linear = 50.0   # mm/s max speed for translations (scaled by joystick)
        self.max_yaw = 30.0      # deg/s max speed for yaw rotation

        # Subscribe to Twist commands
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        # Timer for sending repeated commands at fixed rate
        self.timer = self.create_timer(self.step_time, self.send_command)
        self.joy_active = False  # whether we have a recent command

    def twist_callback(self, msg: Twist):
        # Convert Twist input to target coordinate offsets
        dt = self.step_time
        self.joy_active = True
        dx = msg.linear.x * 1000.0 * dt  # m->mm
        dy = msg.linear.y * 1000.0 * dt
        dz = msg.linear.z * 1000.0 * dt
        dyaw = msg.angular.z * (180.0/3.14159) * dt  # rad->deg
        # Update target coords (assuming base frame axes)
        self.target_coords[0] += dx   # X (mm)
        self.target_coords[1] += dy   # Y (mm)
        self.target_coords[2] += dz   # Z (mm)
        self.target_coords[5] += dyaw # Rz (deg) - adjust end-effector yaw
        # Optional: clamp target_coords to safe workspace bounds here

    def send_command(self):
        if not self.joy_active:
            # No joystick input, do nothing (or could slow to a stop)
            return
        # Send the new target coordinates to the robot at a given speed (e.g. 50% speed, linear path)
        try:
            self.mc.send_coords(self.target_coords, 50, 1)  # 50% speed, mode=1 (linear) [oai_citation:15‡docs.elephantrobotics.com](https://docs.elephantrobotics.com/docs/gitbook-en/7-ApplicationBasePython/7.3_coord.html#:~:text=,Return%20Value%3A%20None)
        except Exception as e:
            self.get_logger().error(f"Failed to send coords: {e}")
        # Reset flag so we only continue sending if new Twist msgs keep coming
        self.joy_active = False

def main():
    rclpy.init()
    node = MyCobotTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
