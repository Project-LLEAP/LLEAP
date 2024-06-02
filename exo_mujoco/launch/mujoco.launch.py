import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Process xacro file
    description_pkg_name = 'exo_description'
    urdf_subpath = 'urdf/test.urdf.xacro'
    urdf_path = os.path.join(get_package_share_directory(description_pkg_name), urdf_subpath)
    robot_desc = xacro.process_file(urdf_path).toxml()
    
    return LaunchDescription([
        Node(
            package='exo_mujoco',
            executable='mujoco_simulator',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
    ])
