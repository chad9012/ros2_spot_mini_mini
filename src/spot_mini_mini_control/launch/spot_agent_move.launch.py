"""
spot_agent_move.launch.py — ROS2 launch for Spot Mini Mini with ARS Agent

Launches:
  1. joy_node          — reads Xbox controller
  2. teleop_node       — converts joy to Twist + JoyButtons
  3. spot_sm_node      — state machine, outputs MiniCmd
  4. spot_agent_commander — BezierGait + IK + ARS Agent → joint trajectories

ROS1 equivalent:
  <node pkg="joy" type="joy_node" .../>
  <node pkg="mini_ros" type="teleop_node" .../>
  <node pkg="mini_ros" type="spot_sm" .../>
  <node pkg="mini_ros" type="spot_pybullet_interface" .../>
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('spot_mini_mini_control')

    # ── Joy Node ──────────────────────────────────────────────────────
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
    teleop_node = Node(
        package='spot_mini_mini_teleop',
        executable='teleop_node',
        name='spot_teleop',
        output='screen'
    )

    # ── State Machine Node ────────────────────────────────────────────
    spot_sm_node = Node(
        package='spot_mini_mini_teleop',
        executable='spot_sm_node',
        name='spot_sm',
        output='screen'
    )

    # ── Spot Agent Commander ────────────────────────────────────────
    # BezierGait + IK + ARS pretrained policy → joint angles
    spot_agent_commander = Node(
        package='spot_mini_mini_control',
        executable='spot_agent_commander',
        name='spot_agent_commander',
        parameters=[PathJoinSubstitution([
            pkg_share, 'config', 'spot_params.yaml'
        ])],
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        spot_sm_node,
        spot_agent_commander,
    ])