# LLEAP

A ROS2-based exoskeleton simulation and control system built with Humble and MoveIt2.

## Prerequisites

- Python 3.10 or higher
- Ubuntu 22.04 (Jammy Jellyfish)

## Installation

### 1. System Setup

**Install Ubuntu 22.04 (Jammy Jellyfish)**

Download from: https://releases.ubuntu.com/jammy/

**Install ROS2 Humble Desktop**

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

**Auto-source ROS2 (Optional)**

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Dependencies

**Gazebo Fortress**

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-gz -y
```

**MoveIt2 for ROS2 Humble**

```bash
sudo apt update
sudo apt install ros-humble-moveit -y
```

### 3. Workspace Setup

**Create ROS2 workspace**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**Clone LLEAP repository**

```bash
git clone https://github.com/LLEAP-Software-Team/LLEAP.git
```

**Verify workspace creation**

```bash
ls
```

### 4. Install Project Dependencies

**Navigate to workspace root**

```bash
cd ~/ros2_ws
```

**Initialize rosdep**

```bash
sudo rosdep init || true
rosdep update
```

**Install dependencies**

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

### 5. Python Virtual Environment

**Install venv tools**

```bash
sudo apt install python3.10-venv python3-pip -y
```

**Create virtual environment**

```bash
cd ~/ros2_ws
python3 -m venv .venv
source .venv/bin/activate
```

**Install Python requirements**

```bash
pip install -r src/LLEAP/requirements.txt
```

> **Note:** Always activate the virtual environment before working on the project:
>
> ```bash
> cd ~/ros2_ws
> source .venv/bin/activate
> ```

### 6. Build the Workspace

**Build with colcon**

```bash
cd ~/ros2_ws
colcon build
```

**Source the workspace**

```bash
source install/setup.bash
```

**Auto-source workspace (Optional)**

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage

### Visualize Exoskeleton in RViz2

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch exo_rviz display.launch.py
```

### Run with MoveIt2

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch exo_moveit demo.launch.py
```

> **Note:** Gazebo integration is currently under refactoring.

## Development Status

- ✅ RViz2 visualization
- ✅ MoveIt2 integration
- 🚧 Gazebo simulation (in progress)
