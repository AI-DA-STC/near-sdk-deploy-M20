# GLIM Integration Implementation Plan for M20 Multi-Robot SLAM

## Project Overview

Integrate the GLIM (GPU LiDAR-Inertial Mapping) package into the M20 multi-robot Gazebo simulation to enable SLAM for each robot using IMU and dual LiDAR sensors.

## Current State Analysis

### Existing Infrastructure
- **Multi-robot launch**: `gazebo_multi_robot.launch.py` spawns M20_A and M20_B
- **Sensor bridges**: IMU and LiDAR topics already bridged to ROS2
- **No SLAM**: `slam_toolbox` dependency exists but is unused

### Available Sensor Topics (per robot)
| Topic | Message Type | Rate |
|-------|--------------|------|
| `/{robot}/IMU` | sensor_msgs/Imu | 200 Hz |
| `/{robot}/LIDAR/FRONT` | sensor_msgs/PointCloud2 | 10 Hz |
| `/{robot}/LIDAR/REAR` | sensor_msgs/PointCloud2 | 10 Hz |

### Sensor Characteristics
- **LiDARs**: 16-layer, 640 horizontal samples, 180° FOV each, 30m range
- **IMU**: Located at base_link, provides orientation + angular velocity + linear acceleration

---

## Implementation Plan

### Phase 1: Configuration Setup

#### Task 1.1: Create GLIM Config Directory Structure

```
src/M20_sdk_deploy/config/glim/
├── M20_A/
│   ├── config.json           # Main config (points to others)
│   ├── config_ros.json       # ROS topic configuration
│   ├── config_sensors.json   # IMU/LiDAR extrinsics
│   ├── config_odometry.json  # Odometry parameters
│   └── config_global_mapping.json
└── M20_B/
    └── (same structure)
```

#### Task 1.2: Create Robot-Specific Configuration Files

**config_ros.json** (M20_A example):
```json
{
  "points_topic": "/M20_A/LIDAR/MERGED",
  "imu_topic": "/M20_A/IMU"
}
```

**config_sensors.json**:
```json
{
  "acc_scale": 1.0,
  "T_lidar_imu": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
}
```

**config_odometry.json** (tuned for 16-layer LiDAR):
```json
{
  "voxel_resolution": 0.25,
  "voxelmap_levels": 2,
  "max_num_keyframes": 15,
  "random_downsample_target": 10000,
  "k_correspondences": 20,
  "keyframe_update_strategy": "OVERLAP"
}
```

---

### Phase 2: Point Cloud Merger Node

Since GLIM expects a single point cloud topic, create a node to merge front and rear LiDAR data.

#### Task 2.1: Create Point Cloud Merger Node

**File**: `src/M20_sdk_deploy/scripts/pointcloud_merger.py`

```python
#!/usr/bin/env python3
"""
Merges front and rear LiDAR point clouds into a single topic for GLIM.
Transforms rear LiDAR points to front LiDAR frame before merging.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('pointcloud_merger')

        # Get robot name from parameter
        self.declare_parameter('robot_name', 'M20_A')
        robot_name = self.get_parameter('robot_name').value

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers
        self.front_sub = self.create_subscription(
            PointCloud2,
            f'/{robot_name}/LIDAR/FRONT',
            self.front_callback,
            sensor_qos
        )
        self.rear_sub = self.create_subscription(
            PointCloud2,
            f'/{robot_name}/LIDAR/REAR',
            self.rear_callback,
            sensor_qos
        )

        # Publisher
        self.merged_pub = self.create_publisher(
            PointCloud2,
            f'/{robot_name}/LIDAR/MERGED',
            sensor_qos
        )

        # Storage for synchronization
        self.front_cloud = None
        self.rear_cloud = None
        self.sync_tolerance = 0.05  # 50ms tolerance

        # Rear LiDAR transform (180° rotation around Z)
        # rear is at (-0.32028, 0, -0.013) with yaw=pi
        # relative to front at (0.32028, 0, -0.013)
        self.rear_to_front_offset = np.array([-0.64056, 0.0, 0.0])

        self.get_logger().info(f'PointCloud merger initialized for {robot_name}')

    def front_callback(self, msg):
        self.front_cloud = msg
        self.try_merge()

    def rear_callback(self, msg):
        self.rear_cloud = msg
        self.try_merge()

    def try_merge(self):
        if self.front_cloud is None or self.rear_cloud is None:
            return

        # Check timestamp synchronization
        front_time = self.front_cloud.header.stamp.sec + \
                     self.front_cloud.header.stamp.nanosec * 1e-9
        rear_time = self.rear_cloud.header.stamp.sec + \
                    self.rear_cloud.header.stamp.nanosec * 1e-9

        if abs(front_time - rear_time) > self.sync_tolerance:
            return

        # Extract points
        front_points = list(pc2.read_points(self.front_cloud,
                                            field_names=('x', 'y', 'z'),
                                            skip_nans=True))
        rear_points = list(pc2.read_points(self.rear_cloud,
                                           field_names=('x', 'y', 'z'),
                                           skip_nans=True))

        # Transform rear points (180° rotation + translation)
        transformed_rear = []
        for p in rear_points:
            # Rotate 180° around Z and translate
            x_new = -p[0] + self.rear_to_front_offset[0]
            y_new = -p[1] + self.rear_to_front_offset[1]
            z_new = p[2] + self.rear_to_front_offset[2]
            transformed_rear.append((x_new, y_new, z_new))

        # Merge
        all_points = front_points + transformed_rear

        # Create merged message
        merged_msg = pc2.create_cloud_xyz32(self.front_cloud.header, all_points)
        self.merged_pub.publish(merged_msg)

        # Clear for next sync
        self.front_cloud = None
        self.rear_cloud = None


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### Task 2.2: Add to CMakeLists.txt

Add the script to the install targets:
```cmake
install(PROGRAMS
  scripts/pointcloud_merger.py
  DESTINATION lib/${PROJECT_NAME}
)
```

---

### Phase 3: GLIM Launch Integration

#### Task 3.1: Create GLIM Launch File

**File**: `src/M20_sdk_deploy/launch/glim_slam.launch.py`

```python
"""
GLIM SLAM Launch File for M20 Multi-Robot

Launches GLIM nodes for each robot with appropriate configuration.

Usage:
  ros2 launch rl_deploy glim_slam.launch.py

  # Or for single robot:
  ros2 launch rl_deploy glim_slam.launch.py robots:=['M20_A']
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os

CONFIG_BASE_PATH = "/home/krishna/Workspace/near-sdk-deploy-M20/src/M20_sdk_deploy/config/glim"

def create_glim_nodes(robot_name):
    """Create GLIM-related nodes for a single robot"""
    config_path = os.path.join(CONFIG_BASE_PATH, robot_name)

    return GroupAction([
        PushRosNamespace(robot_name),

        # Point cloud merger
        Node(
            package='rl_deploy',
            executable='pointcloud_merger.py',
            name='pointcloud_merger',
            output='screen',
            parameters=[{'robot_name': robot_name}],
        ),

        # GLIM ROS node
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim_slam',
            output='screen',
            parameters=[{'config_path': config_path}],
            remappings=[
                ('/points', f'/{robot_name}/LIDAR/MERGED'),
                ('/imu', f'/{robot_name}/IMU'),
            ],
        ),
    ])


def generate_launch_description():
    robots = ['M20_A', 'M20_B']

    # Create nodes for all robots
    robot_nodes = [create_glim_nodes(robot) for robot in robots]

    return LaunchDescription(robot_nodes)
```

#### Task 3.2: Create Combined Launch File (Optional)

**File**: `src/M20_sdk_deploy/launch/gazebo_multi_robot_slam.launch.py`

This will launch both Gazebo simulation and GLIM SLAM together with proper timing.

---

### Phase 4: TF Configuration

GLIM requires proper TF frames. Ensure the following transforms are published:

#### Task 4.1: Verify/Add Static Transforms

The Gazebo bridge should publish:
- `base_link` frame for each robot

Add static transforms for LiDAR frames if needed:

```python
# In launch file, add static transform publishers
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='front_lidar_tf',
    arguments=['0.32028', '0', '-0.013', '0', '0', '0',
               f'{robot_name}/base_link', f'{robot_name}/front_lidar'],
),
```

---

### Phase 5: Testing and Validation

#### Task 5.1: Verify Topic Flow

```bash
# Check merged point cloud
ros2 topic echo /M20_A/LIDAR/MERGED --no-arr

# Check GLIM odometry output
ros2 topic echo /M20_A/glim/odom

# Visualize in RViz2
rviz2 -d config/glim_viz.rviz
```

#### Task 5.2: Validate SLAM Quality

1. Drive robot through environment
2. Monitor odometry drift
3. Check for loop closure detection
4. Verify map consistency

---

### Phase 6: Map Saving and Merging

#### Task 6.1: Configure Map Dumping

Add to each robot's `config_global_mapping.json`:
```json
{
  "dump_path": "/home/krishna/Workspace/near-sdk-deploy-M20/maps/{robot_name}",
  "dump_on_shutdown": true
}
```

#### Task 6.2: Map Merging Procedure (Manual - GUI Required)

Since GLIM map merging is **GUI-only**, follow this procedure:

1. **Stop SLAM nodes** after mapping session
2. **Launch offline viewer**:
   ```bash
   ros2 run glim_ros offline_viewer
   ```
3. **Load sessions**:
   - File → Open New Map → Select M20_A dump directory
   - File → Open Additional Map → Select M20_B dump directory
4. **Align maps**:
   - Select "Indoor" preset (for Edifice environment)
   - Click "Run global registration"
   - Click "Run fine registration" for ICP refinement
5. **Merge**:
   - Click "Create factor"
   - Click "Find overlapping submaps"
   - Click "Optimize" (repeat until converged)
6. **Save merged map**

---

## File Modifications Summary

### New Files to Create

| File | Purpose |
|------|---------|
| `config/glim/M20_A/config.json` | Main GLIM config for M20_A |
| `config/glim/M20_A/config_ros.json` | ROS topic config |
| `config/glim/M20_A/config_sensors.json` | Sensor extrinsics |
| `config/glim/M20_A/config_odometry.json` | Odometry parameters |
| `config/glim/M20_A/config_global_mapping.json` | Global mapping config |
| `config/glim/M20_B/*` | Same structure for M20_B |
| `scripts/pointcloud_merger.py` | LiDAR merger node |
| `launch/glim_slam.launch.py` | GLIM launch file |

### Files to Modify

| File | Modification |
|------|--------------|
| `CMakeLists.txt` | Add scripts and config install targets |
| `package.xml` | Add `glim_ros` dependency |

---

## Execution Order

1. **Create config directory structure**
2. **Create configuration files** for both robots
3. **Implement point cloud merger node**
4. **Create GLIM launch file**
5. **Update CMakeLists.txt and package.xml**
6. **Build workspace**: `colcon build --packages-select rl_deploy`
7. **Test single robot SLAM**
8. **Test multi-robot SLAM**
9. **Document map merging procedure**

---

## Dependencies

Ensure these are installed (already done per user):
- `ros-humble-glim-ros-cuda13.1`
- `libiridescence-dev`
- `libgtsam-points-cuda13.1-dev`

Add to `package.xml`:
```xml
<depend>glim_ros</depend>
<depend>sensor_msgs_py</depend>
```

---

## Potential Issues and Solutions

### Issue 1: Point Cloud Timestamp Sync
**Problem**: Front and rear LiDARs may not be perfectly synchronized
**Solution**: Implemented sync tolerance in merger node (50ms default)

### Issue 2: Sparse 16-Layer LiDAR
**Problem**: May have difficulty with scan matching in some geometries
**Solution**: Increased `k_correspondences` to 20, tuned `voxel_resolution`

### Issue 3: IMU Orientation
**Problem**: GLIM expects specific IMU orientation convention
**Solution**: Verify IMU reads [0, 0, +9.81] at rest; adjust `acc_scale` if needed

### Issue 4: Multi-Robot TF Conflicts
**Problem**: Both robots publishing to same TF tree
**Solution**: Use namespaced frames (`M20_A/base_link`, `M20_B/base_link`)

### Issue 5: Map Merging Automation
**Problem**: No programmatic API for map merging
**Solution**: Document manual GUI procedure; consider scripted workflow using saved poses

---

## Verification Checklist

- [ ] Config files created for both robots
- [ ] Point cloud merger publishing merged clouds
- [ ] GLIM nodes starting without errors
- [ ] Odometry being published (`/{robot}/glim/odom`)
- [ ] Maps being built (visualize in RViz)
- [ ] Map dump directories being populated
- [ ] Map merging procedure documented and tested
