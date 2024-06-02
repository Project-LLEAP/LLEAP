import os
from ament_index_python import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
 
from launch_ros.actions import Node
import xacro
 
# Specify the name of the package and path to xacro file within the package
description_pkg_name = 'exo_description'
moveit_pkg_name = 'exo_moveit'
urdf_subpath = 'urdf/lleap_exo.urdf.xacro'

def generate_launch_description():
    # Gazebo will output all messages, including debug messages from plugins.
    log_level = 'warning'
    log_verbosity = LogInfo(msg='Setting logging gazebo verbosity to ' + log_level)
    set_env_var = SetEnvironmentVariable('GAZEBO_VERBOSITY', log_level)

    # Start Gazebo in a paused state
    set_env_var_paused = SetEnvironmentVariable('GAZEBO_PAUSED', 'true')

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(description_pkg_name), urdf_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(description_pkg_name), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path

    use_sim_time = LaunchConfiguration('use_sim_time')

    exo_moveit_share = get_package_share_directory(moveit_pkg_name)
    new_robot_description = xacro.process_file(os.path.join(exo_moveit_share, 'config', 'lleap_exo.urdf.xacro')).toxml()

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation gazebo clock if true'
    )
        
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': new_robot_description,
        'use_sim_time': use_sim_time}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': os.path.join(
            get_package_share_directory('gazebo_ros'), 'worlds', 'shapes.world')}.items()
    )

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'use_sim_time': use_sim_time},
            new_robot_description,
            os.path.join(exo_moveit_share, "config", "ros2_controllers.yaml")
        ],
        output='screen'
    )
    
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(exo_moveit_share, 'launch'), '/spawn_controllers.launch.py']))
    
    upright_control = Node(
        package='exo_moveit',
        executable='upright_control.py',
        output='screen'
    )

    control_to_gazebo = Node(
        package='exo_moveit',
        executable='control_to_gazebo.py',
        output='screen'
    )
 
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'exo',
                                '-z', '2.15'],
                    output='screen')
    
    reset_world = ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/reset_world', 'std_srvs/srv/Empty'],
                output='screen')

    # Run the node
    return LaunchDescription([
        log_verbosity,
        set_env_var,
        declare_use_sim_time,
        robot_state_publisher,
        reset_world,
        gazebo,
        ros2_control,
        controllers,
        spawn_entity,
        control_to_gazebo,
    ])
 