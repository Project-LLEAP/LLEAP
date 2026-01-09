import os
from ament_index_python import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

# Specify the name of the package and path to xacro file within the package
description_pkg_name = 'exo_description'
urdf_subpath = 'urdf/lleap_exo.urdf.xacro'

def generate_launch_description():
    # Use xacro to process the file with Gazebo Fortress flag
    xacro_file = os.path.join(get_package_share_directory(description_pkg_name), urdf_subpath)

    # Process xacro with fortress flag enabled
    robot_description_raw = xacro.process_file(
        xacro_file,
        mappings={'use_gazebo_fortress': 'true'}
    ).toxml()

    # Set Gazebo resource path
    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(description_pkg_name), 'share')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += pkg_share_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = pkg_share_path

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world = DeclareLaunchArgument(
        name='world',
        default_value='',
        description='Path to world file (empty for default empty world)'
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # Gazebo Sim (Fortress)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': world_file
        }.items()
    )

    # Spawn the robot entity in Gazebo
    # z=1.2 accounts for leg length below torso (legs are ~0.5-0.6m)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'lleap_exo',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.2'
        ],
        output='screen'
    )

    # Bridge between ROS2 and Gazebo topics
    # IMU is bridged from Gazebo to ROS2
    # Joint states need special handling - bridge the full topic path
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/exo/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    # Separate bridge for joint states with remapping
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/lleap_exo/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/lleap_exo/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # Run the node
    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,
        joint_state_bridge
    ])
