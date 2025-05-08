#!/usr/bin/env python3
"""
Launch Gazebo + state estimator + balance controller
"""

import os
from launch               import LaunchDescription
from launch.actions        import (
        ExecuteProcess, DeclareLaunchArgument,
        SetEnvironmentVariable, LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions   import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions     import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix, get_package_share_directory

DESCRIPTION_PKG   = "exo_description"
URDF_FILE         = "urdf/lleap_exo.urdf.xacro"

CONTROL_PKG       = "exo_control" 
STATE_ESTIMATOR_PKG = "exo_state_estimator"
CONTROL_YAML      = "config/lleap_exo_control.yaml"

def generate_launch_description():
    use_fake_hardware_arg = DeclareLaunchArgument("use_fake_hardware", default_value="true",
                                            description="Use fake hardware if true")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    xacro_path = os.path.join(get_package_share_directory(DESCRIPTION_PKG), URDF_FILE)
    robot_description_cmd = Command([
        'xacro ', xacro_path,
        ' use_fake_hardware:=', use_fake_hardware   # ← will be resolved at launch
    ])

    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(DESCRIPTION_PKG), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path

    robot_description = ParameterValue(robot_description_cmd, value_type=str)

    # Gazebo model path so meshes are found
    gazebo_models = os.path.join(
        get_package_share_directory(DESCRIPTION_PKG), "share")
    set_model_path = SetEnvironmentVariable("GAZEBO_MODEL_PATH",
                                            f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:{gazebo_models}")

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true",
                                             description="Use /clock if true")
    use_sim_time     = LaunchConfiguration("use_sim_time")

    rsp = Node(package="robot_state_publisher",
               executable="robot_state_publisher",
               parameters=[{"robot_description": robot_description,
                            "use_sim_time":     use_sim_time}])
    
    state_estimator = Node(
        package=STATE_ESTIMATOR_PKG,
        executable="state_estimator_ukf.py",
        name="state_estimator",
        output="screen",
        parameters=[{
            "urdf_package": DESCRIPTION_PKG,
            "urdf_subpath": URDF_FILE,
            "use_sim_time": use_sim_time,
            "use_fake_hardware": use_fake_hardware,
        }],
        remappings=[
            ("joint_states/filtered", "joint_states"),      # Gazebo raw → filtered topic
            ("imu/filtered",          "imu/data")           # IMU plugin
        ])

    balance_controller = Node(
        package=CONTROL_PKG,
        executable="balance_controller.py",
        name="balance_controller",
        output="screen",
        parameters=[{
            "urdf_package": DESCRIPTION_PKG,
            "urdf_subpath": URDF_FILE,
            "use_sim_time": use_sim_time,
            "use_fake_hardware": use_fake_hardware,
        }],
        remappings=[
            ("joint_states/filtered", "joint_states")
        ])

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen')

    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen')

    spawn = Node(package="gazebo_ros", executable="spawn_entity.py",
                 arguments=["-topic", "robot_description", "-entity", "exo"],
                 output="screen")

    js_broadcaster = TimerAction(
        period=2.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                       "--controller-type", "joint_state_broadcaster/JointStateBroadcaster"],
            output="screen")
    ])

    pos_controller = TimerAction(
        period=1.0,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["vel_controller",
                   "--controller-manager", "/controller_manager",
                   "--param-file", PathJoinSubstitution([get_package_share_directory(CONTROL_PKG),
                                          CONTROL_YAML])],
        output="screen")
    ])

    return LaunchDescription([
        LogInfo(msg="[exo_sim] Launching exoskeleton simulation…"),
        use_sim_time_arg,
        use_fake_hardware_arg,
        set_model_path,
        rsp,
        gzserver,
        gzclient,
        spawn,
        js_broadcaster,
        pos_controller,
        state_estimator,
        balance_controller,
    ])
