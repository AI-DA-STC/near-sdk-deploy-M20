## Overview

This repository uses ROS2 to implement the entire Sim-to-sim and Sim-to-real workflow. Therefore, ROS2 must first be installed on your computer, such as installing [ROS2 Humble](https://docs.ros.org/en/humble/index.html) on Ubuntu 22.04. We've also released an introduction [video](https://www.youtube.com/watch?v=FNaxsDBtD7A), please check it out! Please go through the whole process on a Ubuntu system.

![](/assets/simulation.png)

## Hardware Requirements

**Tested Configuration:**
- GPU: NVIDIA RTX 5090 (Aftershock PC)
- System Memory: 64GB RAM
- VRAM: 25GB

**Observed Resource Usage (Multi-Robot Simulation):**
- VRAM: ~2.2GB
- GPU Utilization: ~16-20%
- Memory Utilization: ~5-6%

## Prerequisites

### ROS2 Humble
Install ROS2 Humble on Ubuntu 22.04:
```bash
# Follow official instructions at:
# https://docs.ros.org/en/humble/Installation.html
```

### Gazebo Ignition Fortress
Install Gazebo Ignition Fortress for simulation:
```bash
# Install Gazebo Ignition Fortress
sudo apt-get update
sudo apt-get install ignition-fortress

# Install ROS2-Gazebo bridge packages
sudo apt-get install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces
```

## Sim-to-sim (Gazebo Ignition Fortress)

### Build

```bash
git clone https://github.com/DeepRoboticsLab/sdk_deploy.git

# Compile
cd sdk_deploy
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rl_deploy --cmake-args -DBUILD_PLATFORM=x86
```

### Single Robot

```bash
# Terminal 1: Launch simulation
source install/setup.bash
ros2 launch rl_deploy gazebo.launch.py

# Terminal 2: Control robot (use namespace)
source install/setup.bash
ros2 run rl_deploy rl_deploy --ros-args -r __ns:=/M20
```

### Multi-Robot

```bash
# Terminal 1: Launch simulation with both robots
source install/setup.bash
ros2 launch rl_deploy gazebo_multi_robot.launch.py

# Terminal 2: Control M20_A
source install/setup.bash
ros2 run rl_deploy rl_deploy --ros-args -r __ns:=/M20_A

# Terminal 3: Control M20_B
source install/setup.bash
ros2 run rl_deploy rl_deploy --ros-args -r __ns:=/M20_B
```

### Topic Structure (Multi-Robot)

The system uses two layers of topics:

**Gazebo Bridge Topics** (standard ROS2 messages from Gazebo):
- `/M20_X/joint_states` - `sensor_msgs/msg/JointState`
- `/M20_X/IMU` - `sensor_msgs/msg/Imu`
- `/M20_X/LIDAR/FRONT` - Front LIDAR sensor
- `/M20_X/LIDAR/REAR` - Rear LIDAR sensor

**DDS Format Topics** (custom messages for rl_deploy controller):
- `/M20_X/JOINTS_DATA` - `drdds/msg/JointsData` (converted from joint_states)
- `/M20_X/IMU_DATA` - `drdds/msg/ImuData` (converted from IMU)
- `/M20_X/JOINTS_CMD` - `drdds/msg/JointsDataCmd` (commands to Gazebo)

The `gazebo_controller_ros2.py` node bridges between these formats.

```bash
# Robot M20_A topics
/M20_A/joint_states       # Gazebo -> Bridge (sensor_msgs/JointState)
/M20_A/IMU                # Gazebo -> Bridge (sensor_msgs/Imu)
/M20_A/LIDAR/FRONT        # Gazebo -> ROS2 (sensor_msgs/LaserScan)
/M20_A/LIDAR/REAR         # Gazebo -> ROS2 (sensor_msgs/LaserScan)
/M20_A/JOINTS_DATA        # Bridge -> rl_deploy (drdds/JointsData)
/M20_A/IMU_DATA           # Bridge -> rl_deploy (drdds/ImuData)
/M20_A/JOINTS_CMD         # rl_deploy -> Gazebo (drdds/JointsDataCmd)

# Robot M20_B topics
/M20_B/joint_states       # Gazebo -> Bridge (sensor_msgs/JointState)
/M20_B/IMU                # Gazebo -> Bridge (sensor_msgs/Imu)
/M20_B/LIDAR/FRONT        # Gazebo -> ROS2 (sensor_msgs/LaserScan)
/M20_B/LIDAR/REAR         # Gazebo -> ROS2 (sensor_msgs/LaserScan)
/M20_B/JOINTS_DATA        # Bridge -> rl_deploy (drdds/JointsData)
/M20_B/IMU_DATA           # Bridge -> rl_deploy (drdds/ImuData)
/M20_B/JOINTS_CMD         # rl_deploy -> Gazebo (drdds/JointsDataCmd)

# Joint force command topics (Gazebo internal)
/model/M20_A/joint/<joint_name>/cmd_force
/model/M20_B/joint/<joint_name>/cmd_force
```

### Keyboard controls

<span style="color: red;">**Note:**</span>
> - When the robot dog stands up, it may become stuck due to self-collision in the simulation. This is not a bug; please try again.
> - **z**: default position
> - **c**: rl control default position
> - **wasd**: forward/leftward/backward/rightward
> - **q,e**: clockwise/counter clockwise


# Sim-to-Real (not tested)
This process is almost identical to simulation-simulation. You only need to add the step of connecting to Wi-Fi to transfer data, and then modify the compilation instructions. The default control mode is currently set to keyboard mode. We will be adding controller support in future updates. Stay tuned.


Please first use the OTA upgrade function in the handle settings to upgrade the hardware to version 1.1.7.

```bash

# computer and gamepad should both connect to WiFi
# WiFi: M20********
# Passward: 12345678 (If wrong, contact technical support)

# scp to transfer files to quadruped (open a terminal on your local computer) password is ' (a single quote)
scp -r ~/sdk_deploy/src user@10.21.31.103:~/sdk_deploy

# ssh connect for remote development, 
ssh user@10.21.31.103
cd sdk_deploy
source /opt/ros/foxy/setup.bash #source ROS2 env
colcon build --packages-select rl_deploy --cmake-args -DBUILD_PLATFORM=arm


sudo su # Root
source /opt/ros/foxy/setup.bash #source ROS2 env
source /opt/robot/scripts/setup_ros2.sh
ros2 service call /SDK_MODE drdds/srv/StdSrvInt32 command:\ 200 # /200 is /JOINTS_DATA topic frequency, recommended below 500 Hz. This value can only be factors of 1000.

# Run
source install/setup.bash
ros2 run rl_deploy rl_deploy

# exit sdk mode：
ros2 service call /SDK_MODE drdds/srv/StdSrvInt32 command:\ 0

# keyboard control
Note: When the robot dog stands up, it may become stuck due to self-collision in the simulation. This is not a bug; please try again.
- z： default position
- c： rl control default position
- wasd：forward/leftward/backward/rightward
- qe：clockwise/counter clockwise
```

