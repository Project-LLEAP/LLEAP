#!/usr/bin/env python3
"""
Launch the exoskeleton in Gazebo + state-estimator + gait_manager.
"""

import os, xacro
from launch               import LaunchDescription
from launch.actions        import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions        import SetEnvironmentVariable
from launch.substitutions  import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions    import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python    import get_package_share_directory, get_package_prefix


def pkg_share(pkg: str) -> str:
    return get_package_share_directory(pkg)


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file   = LaunchConfiguration('world')
    gui          = LaunchConfiguration('gz_gui')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use /clock from Gazebo'),
        DeclareLaunchArgument('world', default_value='empty.world',
                              description='Gazebo world file name (in gazebo/share/worlds)'),
        DeclareLaunchArgument('gz_gui', default_value='true',
                              description='Gazebo client GUI yes/no'),
    ]

    descr_pkg   = 'exo_description'
    xacro_path  = os.path.join(pkg_share(descr_pkg), 'urdf', 'lleap_exo.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    model_path  = os.path.join(get_package_prefix(descr_pkg), 'share')
    set_gz_env  = SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                                         f'{model_path}{os.pathsep}${{GAZEBO_MODEL_PATH:-}}')

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share('gazebo_ros'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={'world': world_file,
                          'gui': gui}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'exo'],
        output='screen'
    )

    state_est_node = Node(
        package='exo_state_estimation',
        executable='state_estimator_pinocchio', 
        name='state_estimator',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    gait_node = Node(
        package='exo_control',
        executable='gait_manager',
        name='gait_manager',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription(
        declare_args +
        [
            set_gz_env,
            robot_state_pub,
            gazebo_launch,
            spawn_entity,
            state_est_node,
            gait_node,
        ]
    )
