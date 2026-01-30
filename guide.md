# GLIM Integration Guide for M20 Multi-Robot SLAM

## Table of Contents
1. [Overview](#overview)
2. [GLIM Architecture](#glim-architecture)
3. [Deep Dive: GLIM's Core Algorithms](#deep-dive-glims-core-algorithms)
   - [Iterative Closest Point (ICP)](#1-iterative-closest-point-icp)
   - [Voxelized GICP (VGICP)](#2-voxelized-gicp-vgicp---glims-core)
   - [IMU Preintegration](#3-imu-preintegration)
   - [Factor Graphs](#4-factor-graphs)
   - [LiDAR-IMU Sensor Fusion](#5-lidar-imu-sensor-fusion-in-glim)
4. [Learning Resources](#learning-resources)
5. [LiDAR-Only vs LiDAR-IMU Estimation](#lidar-only-vs-lidar-imu-estimation)
6. [Key Configuration Parameters](#key-configuration-parameters)
7. [Multi-Robot Map Merging](#multi-robot-map-merging)
8. [Your M20 Sensor Setup](#your-m20-sensor-setup)

---

## Overview

GLIM (GPU LiDAR-Inertial Mapping) is a versatile 3D mapping framework that leverages GPU acceleration for real-time SLAM. It supports:
- Spinning-type LiDARs (Velodyne, Ouster)
- Non-repetitive scan LiDARs (Livox)
- Solid-state LiDARs (Intel Realsense L515)
- RGB-D cameras (Azure Kinect)

---

## GLIM Architecture

### Processing Pipeline

```
┌─────────────────────────────────────────────────────────────────────┐
│                         GLIM Pipeline                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────┐    ┌──────────────┐    ┌─────────────────┐           │
│  │ LiDAR    │───▶│ Preprocessing │───▶│ GPU Odometry    │           │
│  │ Points   │    │ (Voxelization)│    │ (VGICP)         │           │
│  └──────────┘    └──────────────┘    └────────┬────────┘           │
│                                               │                      │
│  ┌──────────┐    ┌──────────────┐             ▼                     │
│  │ IMU Data │───▶│ IMU Preint.  │───▶┌─────────────────┐           │
│  └──────────┘    └──────────────┘    │ Factor Graph    │           │
│                                       │ Optimization    │           │
│                                       └────────┬────────┘           │
│                                                │                     │
│                                                ▼                     │
│                                       ┌─────────────────┐           │
│                                       │ Global Mapping  │           │
│                                       │ (Loop Closure)  │           │
│                                       └─────────────────┘           │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### Key Components

1. **Odometry Estimation**: Uses Voxelized GICP (VGICP) on GPU for scan matching
2. **IMU Preintegration**: Fuses IMU measurements between LiDAR scans
3. **Factor Graph**: GTSAM-based optimization combining odometry and IMU factors
4. **Global Mapping**: Submap-based mapping with loop closure detection

---

## Deep Dive: GLIM's Core Algorithms

This section provides theoretical background on the algorithms GLIM uses. Understanding these concepts helps with debugging, parameter tuning, and appreciating why certain configurations matter.

### 1. Iterative Closest Point (ICP)

#### What is ICP?

ICP is a fundamental algorithm for aligning two point clouds. Given a **source** point cloud and a **target** point cloud, ICP finds the rigid transformation (rotation R and translation t) that best aligns them.

#### The Basic ICP Algorithm

```
Algorithm: Iterative Closest Point
───────────────────────────────────────────────────
Input: Source point cloud P, Target point cloud Q
Output: Transformation T = (R, t)

1. Initialize T = Identity

2. REPEAT until convergence:

   a. FIND CORRESPONDENCES:
      For each point pᵢ in P:
        Find nearest point qᵢ in Q

   b. COMPUTE TRANSFORMATION:
      Minimize: E(R,t) = Σᵢ ||R·pᵢ + t - qᵢ||²
      (Solved via SVD)

   c. APPLY TRANSFORMATION:
      P ← R·P + t

   d. CHECK CONVERGENCE:
      If change in E < threshold: STOP

3. Return accumulated transformation T
───────────────────────────────────────────────────
```

#### ICP Variants

| Variant | Error Metric | Best For |
|---------|--------------|----------|
| **Point-to-Point** | Distance between corresponding points | General use |
| **Point-to-Plane** | Distance to tangent plane at target | Structured environments |
| **Generalized ICP (GICP)** | Distribution-to-distribution matching | Noisy data, higher accuracy |

#### Generalized ICP (GICP)

GICP treats each point as a Gaussian distribution rather than a single point. It minimizes:

```
E(T) = Σᵢ dᵢᵀ (Cᵢᴮ + T·Cᵢᴬ·Tᵀ)⁻¹ dᵢ

Where:
- dᵢ = T·pᵢᴬ - pᵢᴮ  (transformed source minus target)
- Cᵢᴬ, Cᵢᴮ = covariance matrices at each point
```

This "distribution-to-distribution" matching makes GICP more robust to noise and sensor uncertainty.

### 2. Voxelized GICP (VGICP) - GLIM's Core

VGICP is GLIM's primary scan matching algorithm. It combines GICP's accuracy with voxelization for speed.

#### Key Innovation

Instead of finding nearest neighbors for each point (O(n log n)), VGICP:
1. Divides space into voxels
2. Computes a Gaussian distribution per voxel
3. Maps source points directly to voxels (O(1) lookup)

```
┌─────────────────────────────────────────────────────────────────┐
│                    VGICP vs Traditional GICP                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Traditional GICP:                 VGICP:                        │
│  ┌───┐                             ┌───┬───┬───┬───┐            │
│  │ • │──nearest──▶ •               │ μ₁│ μ₂│ μ₃│ μ₄│ ◄─voxel   │
│  │ • │  neighbor   •               ├───┼───┼───┼───┤   grid    │
│  │ • │  search     •               │ μ₅│ μ₆│ μ₇│ μ₈│            │
│  └───┘  (slow)                     └───┴───┴───┴───┘            │
│                                         │                        │
│  For each point:                   For each point:               │
│  Search k-d tree O(log n)          Hash to voxel O(1)           │
│                                    Use voxel's μ, Σ              │
│                                                                  │
│  Speed: ~10 Hz (CPU)               Speed: 30Hz CPU, 120Hz GPU   │
└─────────────────────────────────────────────────────────────────┘
```

#### VGICP Algorithm

```
Algorithm: Voxelized GICP
───────────────────────────────────────────────────
1. PREPROCESSING (Target point cloud):
   - Divide space into voxels of size 'voxel_resolution'
   - For each voxel containing points:
     - Compute mean μ and covariance Σ
     - Store as Gaussian N(μ, Σ)

2. REGISTRATION (Source to Target):
   For each iteration:
     a. For each source point pᵢ:
        - Hash to voxel index
        - Get voxel's distribution N(μᵥ, Σᵥ)

     b. Compute cost:
        E(T) = Σᵢ (T·pᵢ - μᵥ)ᵀ Σᵥ⁻¹ (T·pᵢ - μᵥ)

     c. Optimize T using Gauss-Newton on GPU

3. Return optimal transformation T
───────────────────────────────────────────────────
```

#### Why GPU Acceleration Matters

The cost function and its Jacobian can be computed **independently** for each point, making it embarrassingly parallel:

```
GPU Parallelization:
────────────────────
Point 1 ──▶ Thread 1 ──▶ ┐
Point 2 ──▶ Thread 2 ──▶ │
Point 3 ──▶ Thread 3 ──▶ ├──▶ Reduce ──▶ Update T
   ...         ...       │
Point N ──▶ Thread N ──▶ ┘

Result: 120 Hz registration on modern GPUs
```

### 3. IMU Preintegration

#### The Problem

IMU produces measurements at high frequency (200 Hz for M20). If we added each IMU measurement as a factor in the graph, optimization would be computationally intractable.

#### The Solution: Preintegration

**Preintegration** summarizes all IMU measurements between two keyframes into a single "preintegrated measurement" that can be used as one factor.

```
┌─────────────────────────────────────────────────────────────────┐
│                      IMU Preintegration                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  LiDAR keyframes:    K₁ ─────────────────────── K₂              │
│                       │                          │               │
│  IMU measurements:    ω₁,a₁  ω₂,a₂  ...  ωₙ,aₙ                  │
│                       └──────────┬───────────────┘               │
│                                  │                               │
│                           Preintegrate                           │
│                                  │                               │
│                                  ▼                               │
│                     ┌─────────────────────┐                     │
│                     │  Δpᵢⱼ (position)    │                     │
│                     │  Δvᵢⱼ (velocity)    │  Single             │
│                     │  Δqᵢⱼ (orientation) │  Preintegrated      │
│                     │  Σᵢⱼ  (covariance)  │  Measurement        │
│                     └─────────────────────┘                     │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

#### Preintegration Equations

Given IMU measurements of angular velocity ω and linear acceleration a:

```
IMU Measurement Model:
──────────────────────
ω̃ = ω + bᵍ + ηᵍ     (gyroscope: true + bias + noise)
ã = Rᵀ(a - g) + bᵃ + ηᵃ  (accelerometer: rotated, gravity-removed)

Preintegrated Measurements (between times i and j):
──────────────────────────────────────────────────
Δqᵢⱼ = ∫ᵢʲ q ⊗ [0, ½ω̃]ᵀ dt     (orientation change)

Δvᵢⱼ = ∫ᵢʲ Δqᵢₜ · ã dt          (velocity change)

Δpᵢⱼ = ∫ᵢʲ Δvᵢₜ dt              (position change)
```

The key insight from Forster et al. is that these integrals can be computed **independently of the initial state**, allowing them to be precomputed.

#### Why Preintegration is Powerful

1. **Computational Efficiency**: Hundreds of IMU measurements → 1 factor
2. **Bias Correction**: Preintegrated measurements can be corrected for bias changes without re-integration
3. **Uncertainty Propagation**: Covariance is tracked through integration

### 4. Factor Graphs

#### What is a Factor Graph?

A factor graph is a bipartite graph representing a factorized probability distribution:

```
P(X) = ∏ᵢ fᵢ(Xᵢ)

Where:
- X = {x₁, x₂, ...} are variable nodes (robot poses, velocities, biases)
- fᵢ = factor nodes (constraints from measurements)
```

```
┌─────────────────────────────────────────────────────────────────┐
│                    Factor Graph Structure                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Variable Nodes (circles):     Factor Nodes (squares):          │
│  ○ Robot pose xᵢ               ■ Prior factor                   │
│  ○ Velocity vᵢ                 ■ IMU preintegration factor      │
│  ○ IMU bias bᵢ                 ■ LiDAR odometry factor          │
│                                ■ Loop closure factor             │
│                                                                  │
│  Example SLAM Factor Graph:                                      │
│                                                                  │
│      ■──○──■──○──■──○──■──○                                     │
│      │  x₁ │  x₂ │  x₃ │  x₄                                    │
│      │     │     │     │                                        │
│      ■     ■     ■     ■     ← IMU factors                      │
│      │     │     │     │                                        │
│      ○     ○     ○     ○     ← velocity nodes                   │
│      v₁    v₂    v₃    v₄                                       │
│            └─────┴─────┘                                        │
│              Loop closure                                        │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

#### Factor Types in GLIM

| Factor Type | Connects | Represents |
|-------------|----------|------------|
| **Prior Factor** | x₀ | Initial pose knowledge |
| **IMU Factor** | xᵢ, vᵢ, bᵢ, xⱼ, vⱼ | Preintegrated IMU constraint |
| **Odometry Factor** | xᵢ, xⱼ | VGICP scan matching result |
| **Loop Closure Factor** | xᵢ, xⱼ | Re-observation of same place |

#### Factor Graph Optimization

The goal is to find X* that maximizes the joint probability (or equivalently, minimizes the negative log-likelihood):

```
X* = argmin_X  Σᵢ ||rᵢ(Xᵢ)||²_Σᵢ

Where:
- rᵢ(Xᵢ) = residual (error) of factor i
- ||·||²_Σ = Mahalanobis norm (weighted by covariance)
```

This is solved using **nonlinear least squares** methods:
- Gauss-Newton
- Levenberg-Marquardt
- Dogleg

#### Why Factor Graphs for SLAM?

1. **Modularity**: Easy to add/remove sensor types
2. **Sparsity**: The graph structure leads to sparse matrices → efficient solving
3. **Flexibility**: Can handle delayed measurements, loop closures
4. **Incremental Updates**: iSAM2 allows efficient updates without full re-optimization

### 5. LiDAR-IMU Sensor Fusion in GLIM

GLIM uses **tightly-coupled** fusion where LiDAR and IMU measurements jointly constrain the state.

#### Fusion Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                   LiDAR-IMU Fusion Pipeline                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  IMU @ 200Hz                     LiDAR @ 10Hz                   │
│      │                               │                          │
│      ▼                               ▼                          │
│  ┌────────────┐               ┌─────────────┐                  │
│  │ Preintegr. │               │  VGICP      │                  │
│  │ ω,a → Δp,v,q│              │  Matching   │                  │
│  └─────┬──────┘               └──────┬──────┘                  │
│        │                             │                          │
│        │    IMU Factor               │   Odometry Factor        │
│        │                             │                          │
│        └──────────┬──────────────────┘                          │
│                   │                                              │
│                   ▼                                              │
│           ┌──────────────┐                                      │
│           │ Factor Graph │                                      │
│           │ Optimization │                                      │
│           │   (GTSAM)    │                                      │
│           └──────┬───────┘                                      │
│                  │                                               │
│                  ▼                                               │
│         ┌────────────────┐                                      │
│         │ Optimized State│                                      │
│         │ - Pose (x,y,z,r,p,y)                                  │
│         │ - Velocity                                            │
│         │ - IMU Biases                                          │
│         └────────────────┘                                      │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

#### Fixed-Lag Smoothing

GLIM uses **fixed-lag smoothing** for real-time performance:
- Only optimize over recent keyframes (configurable window)
- Marginalize out old states (convert to prior)
- Balances accuracy vs computational cost

```
Sliding Window Optimization:
────────────────────────────

Full graph:  x₁──x₂──x₃──x₄──x₅──x₆──x₇──x₈
                              └────────────┘
                               Active window

After marginalization:
             [Prior]──x₅──x₆──x₇──x₈
               │
               └── Summarizes information from x₁...x₄
```

### 6. GLIM Paper Key Contributions

Based on the [GLIM paper (arXiv:2407.10344)](https://arxiv.org/abs/2407.10344):

1. **GPU-Accelerated Scan Matching Factors**: Registration error computed in parallel on GPU
2. **Robust Odometry**: Fixed-lag smoothing + keyframe-based registration handles sensor degradation
3. **Multi-Sensor Support**: Extensible to cameras via tightly-coupled visual factors
4. **Global Submap Optimization**: Minimizes registration errors across submaps for loop closure

---

## Learning Resources

### Foundational Papers

| Topic | Paper | Link |
|-------|-------|------|
| **ICP** | Besl & McKay (1992) | [IEEE](https://ieeexplore.ieee.org/document/121791) |
| **GICP** | Segal et al. (2009) | [RSS](http://www.roboticsproceedings.org/rss05/p21.pdf) |
| **VGICP** | Koide et al. (2021) | [ICRA](https://staff.aist.go.jp/shuji.oishi/assets/papers/preprint/VoxelGICP_ICRA2021.pdf) |
| **IMU Preintegration** | Forster et al. (2017) | [TRO](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf) |
| **Factor Graphs** | Dellaert & Kaess (2017) | [Foundations & Trends](https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf) |
| **GLIM** | Koide et al. (2024) | [arXiv](https://arxiv.org/abs/2407.10344) |

### Tutorials & Documentation

| Resource | Description | Link |
|----------|-------------|------|
| **GTSAM Tutorial** | Official factor graph introduction | [gtsam.org](https://gtsam.org/tutorials/intro.html) |
| **What are Factor Graphs?** | Beginner-friendly GTSAM blog | [gtsam.org/blog](https://gtsam.org/2020/06/01/factor-graphs.html) |
| **GTSAM IMU Example** | IMU preintegration walkthrough | [gtbook](https://gtbook.github.io/gtsam-examples/ImuFactorExample101.html) |
| **Graph-Based SLAM** | Classic tutorial paper | [Grisetti et al.](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf) |
| **Factor Graphs Intro** | Beginner-friendly explanation | [sassafras13](https://sassafras13.github.io/FactorGraphs/) |
| **MIT Factor Graph Slides** | SLAM II lecture | [vnav.mit.edu](https://vnav.mit.edu/material/24-SLAM2-FactorGraphsAndMarginalization-slides.pdf) |
| **UCSD Factor Graph SLAM** | University lecture notes | [natanaso](https://natanaso.github.io/ece276a/ref/ECE276A_5_FactorGraphSLAM.pdf) |

### Video Lectures

| Topic | Source | Link |
|-------|--------|------|
| **SLAM Course** | Cyrill Stachniss (Uni Bonn) | [YouTube Playlist](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) |
| **Mobile Robotics** | Stachniss Full Course | [ipb.uni-bonn.de](https://www.ipb.uni-bonn.de/online-training-robotics/) |
| **Visual Navigation** | MIT 16.485 VNAV | [vnav.mit.edu](https://vnav.mit.edu/) |
| **EKF-SLAM Lecture** | Stachniss 2020 | [YouTube](https://www.youtube.com/watch?v=X30sEgIws0g) |

### Code Implementations

| Library | Description | Link |
|---------|-------------|------|
| **fast_gicp** | VGICP with CUDA | [GitHub](https://github.com/koide3/fast_gicp) |
| **small_gicp** | Optimized GICP (header-only) | [GitHub](https://github.com/koide3/small_gicp) |
| **GTSAM** | Factor graph library | [GitHub](https://github.com/borglab/gtsam) |
| **graph_slam** | Multi-factor SLAM example | [GitHub](https://github.com/rising-turtle/graph_slam) |

### Recommended Reading Order

For beginners to SLAM:

1. **Start Here**: [What are Factor Graphs?](https://gtsam.org/2020/06/01/factor-graphs.html) - GTSAM blog post
2. **Graph SLAM**: [Grisetti Tutorial](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf)
3. **IMU Preintegration**: [GTSAM IMU Example](https://gtbook.github.io/gtsam-examples/ImuFactorExample101.html)
4. **VGICP Paper**: [Koide ICRA 2021](https://staff.aist.go.jp/shuji.oishi/assets/papers/preprint/VoxelGICP_ICRA2021.pdf)
5. **GLIM Paper**: [arXiv 2024](https://arxiv.org/abs/2407.10344)

---

## LiDAR-Only vs LiDAR-IMU Estimation

### Comparison Table

| Aspect | LiDAR-Only | LiDAR-IMU |
|--------|------------|-----------|
| **Config File** | `config_odometry_ct.json` | `config_odometry.json` |
| **Robustness to Fast Motion** | Lower | Higher |
| **Drift Characteristics** | More drift in featureless areas | Better drift correction |
| **Computational Load** | Lower | Slightly higher |
| **Calibration Needs** | None | Requires `T_lidar_imu` extrinsics |
| **Setup Complexity** | Simpler | More complex |
| **Best For** | Slow/steady motion, structured environments | Dynamic motion, outdoor, unstructured |

### Tradeoffs in Detail

#### LiDAR-Only Mode
**Pros:**
- No IMU calibration required
- Works with any LiDAR without additional sensors
- Simpler setup and configuration
- No IMU noise/bias issues

**Cons:**
- Struggles with aggressive motion (fast turns, acceleration)
- Higher drift in geometrically degenerate environments (long corridors, open fields)
- No gravity reference for roll/pitch correction
- May fail during rapid rotations

**When to Use:**
- Indoor structured environments
- Slow-moving robots
- When IMU is unavailable or unreliable

#### LiDAR-IMU Mode
**Pros:**
- Better handling of fast/aggressive motion
- Gravity vector provides absolute roll/pitch reference
- IMU preintegration fills gaps between LiDAR scans
- More robust in degenerate geometries
- Better initial pose guess for scan matching

**Cons:**
- Requires accurate IMU-LiDAR extrinsic calibration (`T_lidar_imu`)
- IMU bias estimation adds complexity
- Poor IMU data can degrade performance
- Requires proper `acc_scale` configuration

**When to Use:**
- Outdoor environments
- Fast-moving robots (your M20 quadruped)
- Unstructured or geometrically degenerate areas
- When high accuracy is required

### Recommendation for M20

**Use LiDAR-IMU mode** because:
1. Quadruped robots have dynamic motion with frequent accelerations
2. The M20's wheeled-leg hybrid design produces complex motion patterns
3. IMU data at 200Hz provides excellent motion prediction between 10Hz LiDAR scans
4. Walking/rolling gaits cause body oscillations that IMU can track

---

## Key Configuration Parameters

### Odometry Configuration (`config_odometry.json`)

```json
{
  // Voxel resolution for VGICP (meters)
  // Smaller = more accurate but slower
  // Indoor: 0.1 - 0.25m, Outdoor: 0.5 - 1.0m
  "voxel_resolution": 0.25,

  // Multi-resolution voxelmap levels
  // More levels = handle larger displacements
  "voxelmap_levels": 2,

  // Number of keyframes in local map
  // More = less drift but more memory/compute
  "max_num_keyframes": 15,

  // Downsampling target (points)
  // Lower = faster, Higher = more accurate
  "random_downsample_target": 10000,

  // Correspondence search neighbors
  // Increase for sparse LiDARs (your 16-layer)
  "k_correspondences": 15,

  // Keyframe update strategy
  // OVERLAP: based on scan overlap
  // DISPLACEMENT: based on distance traveled
  // ENTROPY: based on information gain
  "keyframe_update_strategy": "OVERLAP"
}
```

### IMU Configuration (`config_sensors.json`)

```json
{
  // IMU topic
  "imu_topic": "/M20_A/IMU",

  // Acceleration scale factor
  // Set to 1.0 if IMU outputs m/s² (standard ROS)
  // Set to 9.80665 if IMU outputs in 'g' units
  "acc_scale": 1.0,

  // IMU to LiDAR transformation [x, y, z, qx, qy, qz, qw]
  // Critical for LiDAR-IMU fusion accuracy
  "T_lidar_imu": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
}
```

### Global Mapping Configuration (`config_global_mapping.json`)

```json
{
  // Enable/disable IMU in global optimization
  // MUST be false for LiDAR-only mode
  "enable_imu": true,

  // Loop closure detection
  "enable_loop_closure": true,

  // Submap voxel resolution
  "submap_voxel_resolution": 0.5
}
```

### ROS Configuration (`config_ros.json`)

```json
{
  // Point cloud topic
  "points_topic": "/M20_A/LIDAR/FRONT",

  // IMU topic
  "imu_topic": "/M20_A/IMU",

  // TF frame IDs
  "lidar_frame": "base_link",
  "imu_frame": "base_link"
}
```

### Performance Tuning

| Parameter | Low Performance | Balanced | High Accuracy |
|-----------|-----------------|----------|---------------|
| `voxel_resolution` | 0.5 | 0.25 | 0.1 |
| `random_downsample_target` | 5000 | 10000 | 20000 |
| `k_correspondences` | 10 | 15 | 30 |
| `max_num_keyframes` | 10 | 15 | 25 |

---

## Multi-Robot Map Merging

### Important Limitation

**Map merging in GLIM is currently GUI-only.** There is no programmatic API or command-line tool for automated merging. You must use the `offline_viewer` application.

### Merging Workflow

#### Step 1: Collect Map Data

Each robot saves its mapping session to a "dump directory" containing:
- Point cloud submaps
- Pose graph data
- Trajectory information

Enable data dumping in your config:
```json
{
  "dump_path": "/path/to/robot_A_session"
}
```

#### Step 2: Launch Offline Viewer

```bash
ros2 run glim_ros offline_viewer
```

#### Step 3: Load Map Sessions

1. **File → Open New Map**: Load first robot's session (M20_A)
2. **File → Open Additional Map**: Load second robot's session (M20_B)

#### Step 4: Alignment

1. **Choose Registration Preset**:
   - **Indoor**: Smaller search radius, finer resolution
   - **Outdoor**: Larger search radius, coarser initial alignment

2. **Initial Alignment Options**:
   - **Automatic**: Click "Run global registration"
   - **Manual**: Use Gizmo UI to drag/rotate point clouds into rough alignment

3. **Verify Alignment**: Visual inspection of overlapping regions

#### Step 5: Refinement

1. Click **"Run fine registration"** to execute ICP matching
2. Review alignment quality in overlapping areas
3. Repeat if necessary with different parameters

#### Step 6: Create Merge Factor

1. Click **"Create factor"** to add the alignment constraint to the pose graph
2. This links the two maps with the computed transformation

#### Step 7: Optimization

1. Click **"Find overlapping submaps"** to detect all overlap regions
2. Creates matching cost factors for overlapping areas
3. Click **"Optimize"** repeatedly until convergence
4. Monitor the optimization cost - should decrease and stabilize

#### Step 8: Export Merged Map

Save the unified map for later use or visualization.

### Best Practices for Multi-Robot Merging

1. **Ensure Sufficient Overlap**: Robots should have mapped some common areas
2. **Similar Environmental Conditions**: Avoid merging sessions with drastically different lighting/conditions
3. **Pre-validate Individual Maps**: Use "Recover graph" to fix any broken data before merging
4. **Incremental Merging**: For >2 robots, merge pairwise then combine results

### Future Considerations

For automated multi-robot SLAM, consider:
- Running separate GLIM instances per robot
- Publishing maps to shared topics
- Custom node for programmatic alignment using the saved pose graphs
- Alternatively, explore other multi-robot SLAM frameworks (Kimera-Multi, Swarm-SLAM)

---

## Your M20 Sensor Setup

### LiDAR Specifications

Based on `M20.sdf`, both front and rear LiDARs are identical:

| Parameter | Value |
|-----------|-------|
| Type | GPU LiDAR (`gpu_lidar`) |
| Update Rate | 10 Hz |
| Horizontal Samples | 640 |
| Horizontal FOV | 180° (±90°) |
| Vertical Layers | 16 |
| Vertical FOV | 30° (±15°) |
| Min Range | 0.05 m |
| Max Range | 30 m |
| Resolution | 0.01 m |

**Position Offsets:**
- Front LiDAR: (0.32028, 0, -0.013) - facing forward
- Rear LiDAR: (-0.32028, 0, -0.013) - facing backward (yaw = π)

### IMU Specifications

| Parameter | Value |
|-----------|-------|
| Type | IMU (`imu`) |
| Update Rate | 200 Hz |
| Position | (0.0632, -0.0268, -0.0435) relative to base_link |
| Noise | None configured (simulation) |

### Topic Structure (Multi-Robot)

```
Robot M20_A:
├── /M20_A/IMU              (sensor_msgs/Imu @ 200Hz)
├── /M20_A/LIDAR/FRONT      (sensor_msgs/PointCloud2 @ 10Hz)
└── /M20_A/LIDAR/REAR       (sensor_msgs/PointCloud2 @ 10Hz)

Robot M20_B:
├── /M20_B/IMU              (sensor_msgs/Imu @ 200Hz)
├── /M20_B/LIDAR/FRONT      (sensor_msgs/PointCloud2 @ 10Hz)
└── /M20_B/LIDAR/REAR       (sensor_msgs/PointCloud2 @ 10Hz)
```

### Considerations for GLIM Integration

1. **Dual LiDAR Fusion**: You have front and rear LiDARs providing 360° coverage. Options:
   - **Option A**: Merge point clouds into single topic before GLIM
   - **Option B**: Run GLIM on one LiDAR, use other for validation
   - **Option C**: Alternate between LiDARs (not recommended)

2. **16-Layer LiDAR**: Relatively sparse vertical resolution
   - Increase `k_correspondences` to 15-30
   - May need larger `voxel_resolution` for robustness

3. **180° FOV per LiDAR**: Good horizontal coverage but no overlap between front/rear
   - Combined provides full 360° coverage
   - Point cloud merging recommended

4. **IMU at Base Link**: Already colocated with LiDARs
   - `T_lidar_imu` will be based on LiDAR mount positions
   - For front LiDAR: `[-0.2573, 0.0268, 0.0305, 0, 0, 0, 1]` (approximate)

---

## Quick Reference

### Running GLIM

```bash
# Basic launch
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/path/to/config

# With custom topic remapping
ros2 run glim_ros glim_rosnode --ros-args \
  -r /points:=/M20_A/LIDAR/MERGED \
  -r /imu:=/M20_A/IMU
```

### Key Files to Configure

1. `config_ros.json` - ROS topic names
2. `config_odometry.json` - Odometry parameters
3. `config_sensors.json` - IMU settings, extrinsics
4. `config_global_mapping.json` - Global optimization settings

### Verifying IMU Orientation

With robot stationary and z-axis pointing up, IMU should read:
- Linear acceleration: approximately [0, 0, +9.81] m/s²
- Angular velocity: approximately [0, 0, 0] rad/s

If acceleration reads ~1.0 instead of ~9.81, set `acc_scale: 9.80665`
