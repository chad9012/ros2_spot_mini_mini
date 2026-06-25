"""
spot_real.launch.py — ROS2 migration of spot_real.launch

Launches the full real-robot pipeline:
  joy_node → teleop_node → spot_sm_node → spot_commander → real_interface → Teensy
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='spot_joy',
             parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.005}]),
        Node(package='spot_mini_mini_teleop', executable='teleop_node', name='spot_teleop',
             parameters=[{'frequency': 200.0, 'axis_linear_x': 3, 'axis_linear_y': 2,
                          'axis_linear_z': 1, 'axis_angular': 0, 'rb': 7, 'lb': 6,
                          'rt': 4, 'lt': 5, 'updown': 7, 'leftright': 6}]),
        Node(package='spot_mini_mini_teleop', executable='spot_sm_node', name='spot_sm',
             parameters=[{'frequency': 200.0}]),
        Node(package='spot_mini_mini_control', executable='spot_commander', name='spot_commander'),
        Node(package='spot_mini_mini_real', executable='real_interface', name='real_interface',
             parameters=[{'serial_port': '/dev/ttyS0', 'baudrate': 115200, 'publish_rate': 50.0,}],
             )
             ,
    ])