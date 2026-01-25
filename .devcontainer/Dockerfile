FROM mcr.microsoft.com/devcontainers/base:jammy

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository
RUN add-apt-repository universe -y \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble with all necessary packages
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-ament-package \
    python3-pip \
    ros-humble-moveit \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true

# Setup ROS 2 environment for all users
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Set back to dialog for interactive use
ENV DEBIAN_FRONTEND=dialog

# Set the default working directory
WORKDIR /workspaces/LLEAP