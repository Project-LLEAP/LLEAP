from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('exo_control'),
        'config', 'lleap_exo_control.yaml')

    cm = Node(package='controller_manager',
              executable='ros2_control_node',
              parameters=[cfg],
              output='screen')

    load = Node(package='controller_manager',
                executable='spawner',
                arguments=['vel_controller'],
                output='screen')

    return LaunchDescription([cm, load])
