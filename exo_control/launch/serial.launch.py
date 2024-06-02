from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    exo_moveit_launch_dir = os.path.join(get_package_share_directory('exo_moveit'), 'launch')
    exo_moveit_demo_launch = os.path.join(exo_moveit_launch_dir, 'demo.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for communication'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exo_moveit_demo_launch),
        ),

        Node(
            package='your_package_name',
            executable='serialesp32',
            name='joint_state_serial_publisher',
            output='screen',
            arguments=['--serial-port', LaunchConfiguration('serial_port')]
        ),
    ])