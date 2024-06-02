import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

description_pkg_name = 'exo_description'
urdf_subpath = 'urdf/testnew.urdf.xacro'
def generate_launch_description():
    urdf = os.path.join(get_package_share_directory(description_pkg_name), urdf_subpath)
    robot_desc = xacro.process_file(urdf).toxml()

    rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )
    spawn = Node(package='ros_gz_sim', executable='create',
                arguments=['-name', 'exo',
                           '-topic', '/robot_description'],
                 output='screen')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen'
    )
    
    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'robot_description_publisher.rviz')
        ]
    )
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '--verbose', '-r', 'empty.sdf'],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,
        rsp,
        spawn,
        bridge,
        rviz
    ])
