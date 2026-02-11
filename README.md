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
