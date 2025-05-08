from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('exo_description'),
        'urdf', 'lleap_exo.urdf.xacro')
    # xacro expands with use_fake_hardware:=false
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro', urdf_path,
                                                   'use_fake_hardware:=false'])}]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
          {'use_sim_time': False},
          os.path.join(get_package_share_directory('exo_control'),
                       'config', 'lleap_exo_control.yaml')
        ],
        output='both')

    load_controllers = []
    for c in ('velocity_hw_controller', 'joint_state_broadcaster'):
        load_controllers.append(
            Node(package='controller_manager',
                 executable='spawner',
                 arguments=[c])
        )

    return LaunchDescription([robot_state_publisher, controller_manager, *load_controllers])