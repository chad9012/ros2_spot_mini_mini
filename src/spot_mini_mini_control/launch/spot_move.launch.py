"""
spot_move.launch.py — ROS2 migration of spot_move.launch

Launches:
  1. joy_node          — reads Xbox controller
  2. teleop_node       — converts joy to Twist + JoyButtons
  3. spot_sm_node      — state machine, outputs MiniCmd
  4. spot_commander    — BezierGait + IK → joint trajectories

ROS1 equivalent:
  <node pkg="joy" type="joy_node" .../>
  <node pkg="mini_ros" type="teleop_node" .../>
  <node pkg="mini_ros" type="spot_sm" .../>
  <node pkg="mini_ros" type="spot_pybullet_interface" .../>
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ── Joy Node ──────────────────────────────────────────────────────
    # ROS1: <node pkg="joy" type="joy_node" name="spot_joy">
    # ROS2: same package, executable is now "joy_node"
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='spot_joy',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    # ── Teleop Node ───────────────────────────────────────────────────
    # ROS1: <node pkg="mini_ros" type="teleop_node" ...>
    # ROS2: from spot_mini_mini_teleop package
    teleop_node = Node(
        package='spot_mini_mini_teleop',
        executable='teleop_node',
        name='spot_teleop',
        parameters=[{
            'frequency':      200.0,
            'axis_linear_x':  4,
            'axis_linear_y':  3,
            'axis_linear_z':  1,
            'axis_angular':   0,
            'scale_linear':   1.0,
            'scale_angular':  1.0,
            'scale_bumper':   1.0,
            'button_switch':  0,
            'button_estop':   1,
            'rb':             5,
            'lb':             2,
            'rt':             5,
            'lt':             4,
            'updown':         7,
            'leftright':      6,
        }],
        output='screen'
    )

    # ── State Machine Node ────────────────────────────────────────────
    # ROS1: <node pkg="mini_ros" type="spot_sm" ...>
    # ROS2: from spot_mini_mini_teleop package
    spot_sm_node = Node(
        package='spot_mini_mini_teleop',
        executable='spot_sm_node',
        name='spot_sm',
        parameters=[{
            'frequency': 200.0,
            'timeout':   1.0,
        }],
        output='screen'
    )

    # ── Spot Commander ────────────────────────────────────────────────
    # ROS1: spot_pybullet_interface at 600Hz
    # ROS2: spot_commander — BezierGait + IK → joint_trajectory_controller
    spot_commander = Node(
        package='spot_mini_mini_control',
        executable='spot_commander',
        name='spot_commander',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        spot_sm_node,
        spot_commander,
    ])
