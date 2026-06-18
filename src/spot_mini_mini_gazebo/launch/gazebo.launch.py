import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Package Directory Paths
    pkg_description = get_package_share_directory('spot_mini_mini_description')
    pkg_gazebo      = get_package_share_directory('spot_mini_mini_gazebo')
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')

    # File Paths
    xacro_file   = os.path.join(pkg_description, 'urdf', 'spot.urdf.xacro')
    world_file   = os.path.join(pkg_gazebo, 'worlds', 'empty.sdf')

    # Process URDF/Xacro
    robot_description = xacro.process_file(xacro_file).toxml()
    
    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )

    # Force Gazebo Version Env Variable
    os.environ['GZ_VERSION'] = 'harmonic'

    # Include Gazebo Simulation Launch File
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    # Robot State Publisher Node (Configured with namespace & remapping early)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='spot',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('joint_states', '/spot/joint_states')
        ]
    )

    # Robot Spawner Node (Points to the namespaced description topic)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_spot',
        output='screen',
        arguments=[
            '-name', 'spot',
            '-topic', '/spot/robot_description', 
            '-x', '0.0', '-y', '0.0', '-z', '0.5', 
        ]
    )

    # ROS-Gazebo Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )

    # Include the separate Controller Launch File
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'controller.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # MASTER EVENT HANDLER: Trigger the controller spawners once the robot spawns
    load_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[controllers],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        load_controllers_after_spawn,
    ])