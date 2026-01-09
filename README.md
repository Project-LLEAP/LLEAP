# LLEAP
This package is for the ROS2-based exoskeleton simulation developed by LLEAP, a Cal Poly club project.
This goal of this project is to create a biped lower-limb exoskeleton to help someone in need. 

The goal of the simulation is to provide a test bed for control code and to root out design flaws before the expense of manufacturing. 
This is still a preliminary simulation, although work is underway to flush out geometries and physical properties.
This is a collection of packages meant to get the legs running using ROS2 Humble.

## Overview
These are the packages needed to control the legs, visualize it in [rviz2](https://github.com/ros2/rviz/tree/humble), and simulate in 
[Gazebo classic](https://classic.gazebosim.org/).
- `exo_description` xacro URDF file using our custom biped lower-limb exoskeleton STL files
- `exo_gazebo` loads configuration files and runs the simulation software for the legs using gazebo classic and will eventually work in parallel with 
the control code.
- `exo_moveit` runs the manipulation and motion planning software for how to get from one point to another in 3d space.
- `exo_rviz` loads configuration files and runs the visualization software for the legs using rviz2 

## Dependencies
You will need a working installation of [ROS2 Humble](https://docs.ros.org/en/humble/index.html), [Gazebo](http://gazebosim.org/), and 
[moveit2](https://moveit.picknik.ai/humble/index.html)
to use these packages. They have been built and tested only on the following platform in WSL2 however, in theory, it should work for any OS with some 
[extra work](https://docs.ros.org/en/humble/Installation.html). 

- Ubuntu 22.04.02 LTS (Jammy Jellyfish)
- ROS Humble Hawksbill
- Gazebo version 11.10.2

### Python

These packages use Python 3.10. A virtual environment is recommended.

```bash
pip install -r requirements.txt
```

## Local Installation

### Create and configure a workspace

Source your ROS installation:

```bash
source /opt/ros/humble/setup.bash
```

Create or use an existing catkin workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2/src
```

### Cloning the project

To clone this project (essentially all of its packages), type the following into the command line: 

```bash
git clone https://github.com/MaxLewter16/LLEAP.git
```

Next install all dependencies, and build.

```bash
cd ~/ros2_ws/
rosdep install -i --from-path src --rosdistro humble -y
colcon build
```

Source the packages:
```bash
source install/setup.bash
```

The packages should be ready and runnable.

## Usage - Simulation

### exo_rviz
Loads configuration files and runs the visualization software for the legs using Rviz2 
To view the legs in Rviz2 run:
```bash
ros2 launch exo_rviz display.launch.py
```
<img width="1552" alt="rviz" src="https://user-images.githubusercontent.com/35123887/231926841-e22c5501-2011-4126-aabd-e4de87ed1d29.png">

### exo_gazebo

#### Gazebo Classic
Loads configuration files and runs the simulation software for the legs using gazebo classic and will eventually work in parallel with the
[control code](https://classic.gazebosim.org/tutorials?tut=ros_control).
To view the legs in Gazebo classic run:
```bash
ros2 launch exo_gazebo gazebo.launch.py
```
<img width="1552" alt="Screen Shot 2023-04-16 at 12 47 10 AM" src="https://user-images.githubusercontent.com/35123887/232345292-4a579eb0-d3f4-4067-bbc3-fe6b7ccbae59.png">

#### Gazebo Fortress (Ignition Gazebo)
Alternatively, you can use Gazebo Fortress for simulation with full sensor support (IMU + joint states).

**Installation:**
```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

**Running on WSL2:**
Gazebo Fortress requires a workaround for OpenGL rendering on WSL2. Use the provided helper script:
```bash
./exo_gazebo/launch_fortress.sh
```

**Note:** Press the play button (▶) in the bottom left of the Gazebo GUI to start the simulation.

**Available sensors:**
- `/exo/imu` - IMU data at 200 Hz (orientation, angular velocity, linear acceleration)
- `/joint_states` - Joint positions and velocities at 200 Hz

### exo_moveit
After following the source installation of moveit2 you should be able to run the motion planning in Rviz2.
To learn how to use moveit2 follow [this tutorial](https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).
To open moveit2 in Rviz2 run:
```bash 
ros2 launch exo_moveit demo.launch.py
```
<img width="1552" alt="moveit" src="https://user-images.githubusercontent.com/35123887/231926854-d6bccb13-d69b-4d00-bb28-f94bbf0185bf.png">

## Docker
Not recommended but needed for mac developers. First, create a ros2 ws and clone the project like above. 
Use the [graphical docker interface](https://github.com/Tiryoh/docker-ros2-desktop-vnc) for visual development. 
```bash
docker run --name ros2_container -p 6080:80 --security-opt seccomp=unconfined -v [ABSOLUTE_PATH]:/home/ubuntu/ros2_ws --shm-size=512m tiryoh/ros2-desktop-vnc:humble
```
Browse http://127.0.0.1:6080/.

You can access the container remotely using vscode with the docker extension and you can use git locally. To start the container again use:
```bash
docker start ros2_container
```

## License
This software is licensed under the BSD-3-Clause license found in the LICENSE file in the root directory of this source tree.

## Acknowledgements
We have found many tutorials made by [Automatic Addison](https://automaticaddison.com/) very helpful.

We structured our project based on the [curio rover](https://github.com/srmainwaring/curio).
