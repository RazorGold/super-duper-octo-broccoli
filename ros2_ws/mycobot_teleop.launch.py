from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joystick driver
        Node(package='joy', executable='joy_node', name='joy_node', 
             parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.1}]),
        # Teleop Twist mapper
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy',
             parameters=['config/mycobot_joy_teleop.yaml']),
        # MyCobot teleop control node (assuming it's been made into a ROS2 package/executable)
        Node(package='mycobot_teleop', executable='teleop_cartesian', name='mycobot_teleop',
             parameters=[{'port': '/dev/ttyAMA0', 'baud': 115200}])
    ])
