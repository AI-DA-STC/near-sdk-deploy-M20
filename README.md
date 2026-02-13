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

# M20 SDK Deploy - Velodyne GLIM Setup

## 1. Replace Absolute Paths

All launch files and SDF files contain hardcoded absolute paths. You **must** replace them with your own workspace path.

**Search and replace:**

```bash
# Find all occurrences
grep -rn "/home/cjy/deeprobotics_ws" src/M20_sdk_deploy/

# Replace all at once (change /home/YOUR_USER/YOUR_WS to your actual path)
find src/M20_sdk_deploy/ -type f \( -name "*.py" -o -name "*.sdf" -o -name "*.json" \) \
  -exec sed -i 's|/home/cjy/deeprobotics_ws|/home/YOUR_USER/YOUR_WS|g' {} +
```

**Files that need path updates:**

| File | Lines | Variables |
|---|---|---|
| `launch/gazebo.launch.py` | 10, 12, 32 | `MODEL_PATH`, `WORLD_FILE`, `robot_sdf` |
| `launch/gazebo_velodyne.launch.py` | 10, 12, 32 | `MODEL_PATH`, `WORLD_FILE`, `robot_sdf` |
| `launch/gazebo_multi_robot.launch.py` | 34, 36, 38 | `MODEL_PATH`, `WORLD_FILE`, `ROBOT_SDF` |
| `launch/gazebo_multi_robot_slam.launch.py` | 34-37 | `MODEL_PATH`, `WORLD_FILE`, `ROBOT_SDF`, `CONFIG_BASE_PATH` |
| `launch/glim_slam.launch.py` | 20 | `CONFIG_BASE_PATH` |
| `model/Edifice_simple/edifice_simple.sdf` | 35 | Depot_simple `<uri>` |
| `config/glim/M20_A/config_global_mapping_gpu.json` | 51 | `dump_path` |
| `config/glim/M20_B/config_global_mapping_gpu.json` | 51 | `dump_path` |

---

## 2. Build the ROS2 Workspace

```bash
cd ~/deeprobotics_ws
colcon build --packages-select rl_deploy
source install/setup.bash
```

---

## 3. Launch Gazebo Simulation

**Original M20 (front + rear lidar):**
```bash
ros2 launch rl_deploy gazebo.launch.py
```

**M20 with Velodyne HDL-32E:**
```bash
ros2 launch rl_deploy gazebo_velodyne.launch.py
```

---

## 4. GLIM SLAM with Docker (GPU)

### 4.1 Pull and Run GLIM Docker

```bash
# Allow X11 forwarding
xhost +local:docker

# Run GLIM container (mount config directly to install path to avoid manual copy)
docker run -it --name glim_docker --gpus all \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /path/to/glim_docker/glim/config:/root/ros2_ws/install/glim/share/glim/config \
  -v /path/to/glim_docker/glim_ros2:/glim_ros2_src \
  koide3/glim_ros2:humble
```

> **Important:** Mount directly to `/root/ros2_ws/install/glim/share/glim/config` (the install path). If you mount to `/glim_src` instead, GLIM will **not** read your config changes — you would have to manually copy them:
> ```bash
> docker exec glim_docker cp /glim_src/config/config_ros.json /root/ros2_ws/install/glim/share/glim/config/
> docker exec glim_docker cp /glim_src/config/config_sensors.json /root/ros2_ws/install/glim/share/glim/config/
> ```

### 4.2 GLIM Config Changes

**`config_ros.json`** — Update ROS2 topics:

| Parameter | Default (Ouster) | M20 Velodyne |
|---|---|---|
| `imu_topic` | `/os_cloud_node/imu` | `/M20/IMU` |
| `points_topic` | `/os_cloud_node/points` | `/M20/LIDAR/VELODYNE` |

**`config_sensors.json`** — Update IMU noise and LiDAR-IMU extrinsic:

| Parameter | Default (Ouster) | M20 Velodyne (Simulation) |
|---|---|---|
| `imu_acc_noise` | `0.05` | `0.001` |
| `imu_gyro_noise` | `0.02` | `0.001` |
| `imu_int_noise` | `0.001` | `0.0001` |
| `imu_bias_noise` | `1e-5` | `1e-7` |
| `T_lidar_imu` | `[0.006, -0.012, 0.008, 0, 0, 0, 1]` | `[-0.2368, -0.0268, -0.1885, 0, 0, 0, 1]` |

> **Note on IMU noise:** The Gazebo simulated IMU has `<noise type='none'/>` (perfect, zero noise) and `update_rate` set to `200` Hz (in `M20_velodyne.sdf`). Setting high noise values causes GLIM to under-trust IMU and drift. For **real hardware**, use the sensor's actual noise specs instead.

> **Note on T_lidar_imu:** This is in TUM format `[x, y, z, qx, qy, qz, qw]`. It transforms points from IMU frame to LiDAR frame. Calculated from:
> - IMU pose on M20: `(0.0632, -0.0268, -0.0435)`
> - Velodyne pose on M20: `(0.3, 0, 0.145)`
> - T_lidar_imu = Velodyne_pose - IMU_pose = `(-0.2368, -0.0268, -0.1885)`, identity rotation

### 4.3 Run GLIM inside Docker

```bash
# Inside the container
source /root/ros2_ws/install/setup.bash
ros2 launch glim glim.launch.py
```

### 4.4 Visualize in RViz (on host)

```bash
# Point cloud topic
ros2 topic echo /M20/LIDAR/VELODYNE

# In RViz: Add PointCloud2, set topic to /M20/LIDAR/VELODYNE
# GLIM outputs: /glim/map (map point cloud), /glim/odom (odometry)
```

---

## 5. Troubleshooting

### Gazebo GUI opens but no models visible

This is a known rendering issue on dual-GPU systems (Intel iGPU + NVIDIA dGPU).

**Solution: Force NVIDIA rendering**
```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
ros2 launch rl_deploy gazebo_velodyne.launch.py
```

**Verify which GPU is being used:**
```bash
glxinfo | grep "OpenGL renderer"
# Should show: NVIDIA GeForce RTX XXXX
```

**If still not working:**
- Test with: `ign gazebo shapes.sdf` — if this works, the issue is with the world SDF, not the GPU
- The `edifice_simple.sdf` had a deprecated `GzScene3D` GUI plugin which has been removed. If the `<gui>` section returns, remove it and let Gazebo use its default GUI

### Velodyne bridge has no data / RViz shows no point cloud

The Gazebo bridge topic must include the **full world-namespaced path**:

```
# Wrong (won't work):
/model/M20/link/base_link/sensor/velodyne_hdl32e/scan/points

# Correct:
/world/Edifice/model/M20/link/base_link/sensor/velodyne_hdl32e/scan/points
```

**Verify the actual topic name:**
```bash
ign topic -l | grep velodyne
```

### GLIM drift when robot is stationary

Possible causes:
1. **IMU noise mismatch** — Simulated IMU has zero noise but GLIM expects real-sensor noise levels. Lower the noise parameters (see Section 4.2)
2. **Insufficient LiDAR features** — Simple environments lack geometric features for scan matching. Try a more detailed world SDF
3. **T_lidar_imu wrong** — Verify the IMU and LiDAR poses in your SDF file and recalculate

### `docker exec` command not found

Run `docker exec` from the **host terminal**, not from inside the container.

### colcon build fails

```bash
cd ~/deeprobotics_ws
rm -rf build/ install/ log/
colcon build --packages-select rl_deploy
source install/setup.bash
```

### GLIM config not taking effect in Docker

GLIM reads config from the **install** directory, not the source:
- Install path: `/root/ros2_ws/install/glim/share/glim/config/`
- Source mount: `/glim_src/config/` (NOT used at runtime)

Either mount directly to the install path (recommended) or manually copy after changes:
```bash
docker exec glim_docker cp /glim_src/config/config_ros.json /root/ros2_ws/install/glim/share/glim/config/
docker exec glim_docker cp /glim_src/config/config_sensors.json /root/ros2_ws/install/glim/share/glim/config/
```

---


## 6. Key Topics Reference

| Topic | Type | Description |
|---|---|---|
| `/M20/IMU` | `sensor_msgs/msg/Imu` | IMU data (200Hz) |
| `/M20/LIDAR/VELODYNE` | `sensor_msgs/msg/PointCloud2` | Velodyne HDL-32E point cloud (10Hz) |
| `/M20/LIDAR/FRONT` | `sensor_msgs/msg/PointCloud2` | Front lidar (original M20 only) |
| `/M20/LIDAR/REAR` | `sensor_msgs/msg/PointCloud2` | Rear lidar (original M20 only) |
| `/glim/map` | `sensor_msgs/msg/PointCloud2` | GLIM map output |
| `/glim/odom` | `nav_msgs/msg/Odometry` | GLIM odometry output |

---

## 7. File Structure

```
M20_sdk_deploy/
├── launch/
│   ├── gazebo.launch.py              # Original M20 launch
│   ├── gazebo_velodyne.launch.py     # M20 + Velodyne HDL-32E launch
│   ├── gazebo_multi_robot.launch.py  # Multi-robot launch
│   ├── gazebo_multi_robot_slam.launch.py
│   └── glim_slam.launch.py          # GLIM SLAM launch (native, non-Docker)
├── model/
│   ├── M20_urdf/urdf/
│   │   ├── M20.sdf                   # Original robot (front + rear lidar)
│   │   └── M20_velodyne.sdf          # Velodyne variant (HDL-32E on head)
│   ├── Edifice_simple/
│   │   └── edifice_simple.sdf        # World file (warehouse environment)
│   └── Depot_simple/                 # Warehouse mesh model
├── config/glim/                      # GLIM config for native launch
├── policy/                           # RL policy files
└── scripts/                          # Utility scripts
```
