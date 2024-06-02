import os
from ament_index_python import get_package_share_directory, get_package_prefix
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import xacro


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("lleap_exo", package_name="exo_moveit").to_moveit_configs()
    ld = generate_demo_launch(moveit_config)
    # Gazebo launch setup
    log_level = 'warning'
    ld.add_action(LogInfo(msg=f'Setting logging gazebo verbosity to {log_level}'))
    ld.add_action(SetEnvironmentVariable('GAZEBO_VERBOSITY', log_level))

    xacro_file = os.path.join(get_package_share_directory('exo_description'), 'urdf/lleap_exo.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    pkg_share_path = os.pathsep + os.path.join(get_package_prefix('exo_description'), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share_path

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        )
    )

    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw, 'use_sim_time': use_sim_time}]
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            )
        )
    )

    ld.add_action(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'exo', '-z', '3.0'],
            output='screen'
        )
    )

    return ld
