import os
from ament_index_python import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
 
from launch_ros.actions import Node
import xacro
 
# Specify the name of the package and path to xacro file within the package
description_pkg_name = 'exo_description'
urdf_subpath = 'urdf/testclassic.urdf.xacro'

def generate_launch_description():
    # Gazebo will output all messages, including debug messages from plugins.
    log_level = 'warning'
    log_verbosity = LogInfo(msg='Setting logging gazebo verbosity to ' + log_level)
    set_env_var = SetEnvironmentVariable('GAZEBO_VERBOSITY', log_level)
    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(description_pkg_name), urdf_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(description_pkg_name), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation gazebo clock if true'
    )
        
    # Configure the node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': use_sim_time}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']))
 
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'exo',
                                '-z', '3.0'],
                    output='screen')

    # Run the node
    return LaunchDescription([
        log_verbosity,
        set_env_var,
        declare_use_sim_time,
        robot_state_publisher,
        gazebo,
        spawn_entity
    ])
 