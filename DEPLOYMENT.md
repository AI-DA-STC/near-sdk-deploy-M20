# M20 deployment — robot-agnostic nav_core + M20 bridge

Two containers from **one image** on the GOS host (10.21.31.104). Nothing is
installed on AOS (10.21.31.103) — the bridge talks to stock firmware over the
UDP Inspection Protocol + direct DDS.

```
AOS 10.21.31.103 (stock, untouched)     ┌ m20_bridge container ─────────────┐  ┌ nav_core container ─────────────┐
 UDP :30000 ─1002/4 joints,vel,estop──► │ m20_udp_node → /joint_states       │  │ p2l /lidar/points → /scan       │
            ◄─heartbeat 1Hz / Cmd21 20Hz│   → /m20/telemetry, /robot/ready   │  │ amcl (map→odom), costmaps       │
 DDS /IMU_YESENSE 200Hz ──────────────► │ restamp /IMU_YESENSE→/imu (offset) │  │ ekf /odom (+/imu) → odom→base   │
 DDS /LIO_ODOM ───────────────────────► │ restamp /LIO_ODOM→/odom  (offset)  │─►│ route_server(graph)→orchestrator│
 rsdriver (GOS-local) /LIDAR/POINTS ───►│ restamp /LIDAR/POINTS→/lidar/points│◄─│ nav2 → /cmd_vel                 │
                                        │ static TF base_link→lidar_link     │  └─────────────────────────────────┘
 laptop RViz (10.21.34.x) /goal_pose ──────────────────────────────────────────►  route_server
                                        │ RSP: M20 URDF → /robot_description │        (viz: /robot_description, TF)
                                        └────────────────────────────────────┘
```

**Generic contract** (nav_core knows only these — port to another robot by
writing a new `<robot>_bridge` package + launch; nav_core is untouched):

| dir | topic | type | produced by |
|-----|-------|------|-------------|
| IN  | `/lidar/points` | PointCloud2 | bridge (rsdriver rename) |
| IN  | `/imu`   | Imu      | bridge (restamp /IMU_YESENSE) |
| IN  | `/odom`  | Odometry | bridge (restamp /LIO_ODOM) |
| IN  | `/joint_states` + `/robot_description` + TF `base_link→lidar_link` | | bridge |
| IN  | `/goal_pose`, `/initialpose` | | laptop RViz |
| OUT | `/cmd_vel` | Twist | nav_core DWB → bridge → UDP Cmd 21 |

All `/*` stamps are on the **GOS clock** (AOS clocks are years off; the bridge
restamps in `offset` mode, preserving source dt for GLIM/EKF).

---

# Stage 0 — laptop simulation shakedown (do this FIRST, before you build or deploy)

`navigation.launch.py` (nav_core) is **byte-identical** in sim and on the robot —
`sim:=true` only swaps in rf2o (`/odom`), the Gazebo sensor-frame static TFs, the
sim EKF variant, and sim time in place of the hardware bridge. So running it in
Gazebo on your laptop exercises the entire nav_core black box — routing, costmaps,
planners, DWB tuning, the orchestrator, AMCL/EKF, and the RViz goal flow — against
the **exact same map, graph, and params** you will ship. Get this green before you
spend a build+transfer cycle or put weight on the real robot.

**Where it runs:** the dev container (`.devcontainer/` — `ros:humble-desktop-full`
+ `ros-gz` + Nav2, CycloneDDS, `--gpus all`) or a native Ubuntu 22.04 + Humble
*desktop* box (see the README apt list). The slim `m20-deploy:humble` image
**cannot** run this — it has no Gazebo/RViz. A GPU is required (the Velodyne is a
Gazebo `gpu_lidar`); this laptop's RTX is fine and the devcontainer already passes
`--gpus all`.

**Who supplies each generic-contract topic (the seam sim replaces):**

| generic input | in simulation (laptop) | on the robot |
|-----|-----|-----|
| `/lidar/points` | Gazebo `gpu_lidar` → velodyne bridge | m20_bridge (rsdriver rename) |
| `/imu`          | Gazebo IMU sensor → bridge | restamp `/IMU_YESENSE` |
| `/odom`         | **rf2o** scan-matching (sim-only node) | restamp `/LIO_ODOM` (onboard LIO) |
| `/joint_states` + `/robot_description` + TF | Gazebo joint bridge + RSP + static TFs | m20_bridge |
| `/cmd_vel` consumer | `m20_cmd_vel_bridge` → `/M20/cmd_vel` → `rl_deploy` RL policy → Gazebo joints | m20_bridge → UDP Cmd 21 → firmware |
| clock | Gazebo `/clock` (`use_sim_time:=true`) | GOS wall clock (bridge restamps) |

**What sim does NOT cover** (still gated by the on-robot T1–T7): the m20_bridge
UDP + direct-DDS transport, its restamp/offset clock handling and QoS matching
(T1–T2); Cmd 21 actuation, axis scale/deadband calibration, and firmware
stream-loss behaviour (T4–T6); real lidar near-field blind ring / body-pitch
effects and real `/IMU_YESENSE` rates; OTA topic renames. A green sim proves
**nav_core**, nothing about the M20 transport.

### TL0 — environment + build sanity
- Open the repo in the dev container (VS Code → "Reopen in Container"), or on a native desktop-Humble box.
- `git submodule update --init --recursive` — pulls `rf2o_laser_odometry`, which supplies `/odom` in sim (missing → TL2 has no odometry).
- Build + source, then confirm the launch files parse:
  ```bash
  colcon build --cmake-args -DBUILD_PLATFORM=x86
  source install/setup.bash
  ros2 pkg list | grep rl_deploy                              # package present
  ros2 launch rl_deploy gazebo_velodyne.launch.py --print     # parses
  ros2 launch rl_deploy navigation.launch.py sim:=true --print
  ```
- If GUIs never appear later, run `xhost +local:` on the host (the devcontainer already mounts the X11 socket + sets `DISPLAY`).

### TL1 — Gazebo bringup = the sim "bridge" (Terminal 1)
```bash
ros2 launch rl_deploy gazebo_velodyne.launch.py
```
- The Ignition window opens on the Edifice world; the M20 spawns at ~8 s and the sensor bridges + `gazebo_controller` start at ~15 s. The robot should settle roughly upright on the floor.
- In a second shell (`source install/setup.bash`), confirm the generic inputs flow:
  ```bash
  ros2 topic hz /lidar/points   # ~10 Hz
  ros2 topic hz /imu            # ~200 Hz
  ros2 topic hz /joint_states   # steady
  ros2 topic echo /clock --once # sim time is ticking
  ```
- Silent `/lidar/points` ⇒ the GPU lidar isn't rendering: check `nvidia-smi` sees the GPU inside the container and the Gazebo window is actually open (GPU-lidar needs a live GL/EGL context).

### TL2 — nav_core, sim mode (Terminal 2)
```bash
ros2 launch rl_deploy navigation.launch.py sim:=true    # sim is the default; explicit is clearer
```
Same launch file, params, map (`edifice_SLAM_v0_2d.yaml`) and graph (`graph.gml`) you will deploy.
- Checks:
  ```bash
  ros2 topic hz /scan                       # pointcloud_to_laserscan alive
  ros2 topic hz /odom                       # rf2o odometry
  ros2 run tf2_ros tf2_echo map base_link   # full map->odom->base_link chain resolves
  ros2 topic echo /amcl_pose --once         # AMCL is publishing a pose
  ```
- Every Nav2 lifecycle node should reach **active** with no stall (the devcontainer runs CycloneDDS specifically to dodge the in-container FastDDS lifecycle-service hang noted in `Dockerfile.deploy`).

### TL3 — RViz + localization (Terminal 3)
```bash
rviz2 -d custom_nav2.rviz --ros-args -p use_sim_time:=true   # from the repo root
```
- The map renders, the robot model sits at its pose, and `/scan` + the costmaps overlay the map. AMCL boots at map `(0,0,0)` (`set_initial_pose: true`).
- **If the scan doesn't line up with the walls**, click **2D Pose Estimate** and drop the arrow where the robot actually is, then confirm the scan snaps onto the map. (The Gazebo spawn is world `(12, 4.5)`, which need not equal map `(0,0)`, so a one-time estimate may be needed.)

### TL4 — stand, RL-control, and the `/cmd_vel` seam by hand (Terminals 4–5)
The sim stand-in for the robot's actuation path. `navigation.launch.py` emits `/cmd_vel`, but **nothing consumes it** until you start both the RL policy and the cmd_vel bridge — neither is auto-launched.
```bash
# Terminal 4 — RL locomotion policy. Subscribes to the ABSOLUTE topics
# /M20/cmd_vel and /M20/target_mode (the __ns remap does not change them).
ros2 run rl_deploy rl_deploy --ros2-cmd --ros-args -r __ns:=/M20

ros2 topic pub -1 /M20/target_mode std_msgs/msg/Int32 "{data: 1}"   # 1 = StandingUp   -> robot stands
ros2 topic pub -1 /M20/target_mode std_msgs/msg/Int32 "{data: 6}"   # 6 = RLControlMode -> policy holds it upright
```
```bash
# Terminal 5 — /cmd_vel -> /M20/cmd_vel bridge (NOT in any launch file; clamps to hw limits + 0.5 s watchdog)
ros2 run rl_deploy m20_cmd_vel_bridge.py
```
- Drive a manual step and watch the seam:
  ```bash
  ros2 topic echo /M20/cmd_vel &                                    # clamped mirror of your command
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
  ```
  → `/M20/cmd_vel` tracks it and the robot walks forward. Ctrl-C the publisher → the bridge watchdog zeros velocity within 0.5 s → robot stops. (`target_mode 2` = JointDamping to relax/sit.)

### TL5 — end-to-end autonomy (the sim analog of T7)
Everything from TL1–TL4 running, policy in mode `6`, bridge up. In RViz click **2D Goal Pose** on a reachable point.
```
/goal_pose -> route_server (graph.gml) -> linear_orchestrator -> planner -> smoother
           -> controller (DWB) -> /cmd_vel -> m20_cmd_vel_bridge -> /M20/cmd_vel
           -> rl_deploy policy -> Gazebo -> robot walks the route (StoppedGoalChecker ends it)
```
- Watch DWB track the path without corner-cutting or oscillation and stop cleanly at the goal — the exact behaviour you tuned for the robot, minus the bridge.
- Orchestrator controls:
  ```bash
  ros2 service call /linear_orchestrator/pause std_srvs/srv/Trigger   # hold (manual /cmd_vel still works)
  ros2 service call /linear_orchestrator/stop  std_srvs/srv/Trigger   # cancel
  # resume: re-publish target_mode 1 then 6
  ```

**Map/graph pipeline in sim (optional):** to exercise mapping and not just navigation, record `/lidar/points` + `/imu` while teleop-walking the sim, then run the same offline chain as the robot — GLIM → `ply_to_2d_map.py` → SWAGGER `graph.gml` (README "Usage", and the **GLIM mapping** section below). Drop the outputs in `maps/` and re-run TL2–TL5 against them.

**Exit gate.** Sim is green when TL5 walks a route with clean DWB tracking, AMCL holding, and no lifecycle stalls. That validates the **nav_core** you are about to ship (byte-identical on the robot). It does **not** clear the M20 bridge or firmware actuation — those are T1–T6 below, and `enable_tx:=true` stays off until they pass on the real robot.

---

# Stage 1 — deploy to the robot

## Build + offline transfer

Two hops, two mechanisms:
- **WS → AOS**: `docker pull` from a registry running on the laptop — shows
  per-layer progress and moves only changed layers on each rebuild.
- **AOS → GOS**: manual `docker save | scp | load` (GOS is segmented behind AOS
  and has no internet — see the note below).

### One-time setup
QEMU emulators (cross-building arm64 on x86; resets on reboot unless persisted):
```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
```
buildx builder (the default docker driver can't cross-build reliably):
```bash
docker buildx create --name multiarch --driver docker-container --use --bootstrap
```
A local image registry on the laptop (persists across reboots):
```bash
docker run -d --restart=always -p 5000:5000 --name registry registry:2
```
On **AOS**, trust the laptop's plain-HTTP registry, then restart docker:
```bash
# /etc/docker/daemon.json   (<WS_IP> = laptop's address on the 10.21.31.x LAN)
{ "insecure-registries": ["<WS_IP>:5000"] }
sudo systemctl restart docker
```

### Each change: build on WS, pull on AOS
```bash
# WS (laptop): build arm64, then push to the local registry.
# NOTE: --load into the host daemon first, THEN push — buildx's --push can't
# see localhost:5000 (buildkit runs in its own container).
docker buildx build --platform linux/arm64 -f Dockerfile.deploy -t m20-deploy:humble --load .
docker tag  m20-deploy:humble localhost:5000/m20-deploy:humble   # localhost = auto-insecure, no config
docker push localhost:5000/m20-deploy:humble                     # progress + only changed layers
```
```bash
# AOS: pull the arm64 image (progress bars; only changed layers download).
docker pull --platform linux/arm64 <WS_IP>:5000/m20-deploy:humble
docker tag  <WS_IP>:5000/m20-deploy:humble m20-deploy:humble     # restore the clean name
```

### AOS → GOS (manual — GOS reaches only AOS)
```bash
SIZE=$(docker image inspect m20-deploy:humble --format='{{.Size}}')
docker save m20-deploy:humble | pv -s "$SIZE" | gzip > m20-deploy.tar.gz # on AOS
scp m20-deploy.tar.gz user@10.21.31.104:/tmp/                    # AOS → GOS (direct)
ssh user@10.21.31.104 'gunzip -c /tmp/m20-deploy.tar.gz | docker load'
```
> GOS (10.21.31.104) sits behind AOS with no internet, so it can't pull from the
> laptop. To make this hop incremental too, run a second `registry:2` on AOS,
> push to it, and `docker pull` from GOS — at the cost of docker + a registry on
> AOS.

## Run

```bash
# on GOS, from the repo (docker-compose.yml)
docker compose up m20_bridge          # bridge only (tests T1–T3)
docker compose up                     # both containers (nav)
ENABLE_TX=true docker compose up      # arm UDP actuation (ONLY after T4–T6)
```

Maps + SWAGGER graph live in the host `~/maps` (mounted into nav_core).

---

## On-robot test plan — unit by unit (T0–T7)

Only after **Stage 0** is green. Run in order. **Do not set `enable_tx:=true`
until T4–T6 pass.** Keep a person on the hardware e-stop for every powered test.

### T0 — image sanity (bench, no robot)
- `docker compose config` parses; `docker run --rm m20-deploy:humble ros2 pkg list | grep -E 'rl_deploy|m20_bridge'` shows both.
- `docker run --rm m20-deploy:humble ros2 launch rl_deploy navigation.launch.py sim:=false --print` and `... m20_bridge m20_bridge.launch.py --print` both succeed (run from `/root/ros_ws`).

### T1 — DDS visibility from inside the bridge container (robot powered, on stand)
Gate for everything DDS. In the `m20_bridge` container shell:
- `ros2 topic list` shows `/IMU_YESENSE`, `/LIO_ODOM`, `/LIDAR/POINTS`.
- `ros2 topic hz /IMU_YESENSE` (expect ~200 Hz — the diagram said 20; confirm the real rate), `/LIO_ODOM`, `/LIDAR/POINTS` (~10 Hz).
- `ros2 topic info -v /IMU_YESENSE /LIO_ODOM /LIDAR/POINTS` → record **reliability** + **type** (confirm `/LIO_ODOM` is `nav_msgs/Odometry`). If a `restamp_*` node logs `INCOMPATIBLE QoS`, fix `reliability` in `m20_bridge.launch.py` and rebuild.
- If discovery is flaky: widen the FastDDS initial-peer range / drop SHM in `config/fastdds_profile_gos.xml`. Confirm `--ipc=host` is set (SHM with the host rsdriver).

### T2 — bridge RX only (`enable_tx:=false`)
- `ros2 topic hz /imu /odom /lidar/points` match the source rates.
- `ros2 topic echo /imu --field header.stamp` ≈ `date +%s` (offset restamp working); consecutive `/imu` dt ≈ 5 ms (not bursty → GLIM-safe).
- `ros2 topic echo /m20/telemetry` shows MotionState/HES/Gait/Version; `/robot/ready` publishes.

### T3 — joints + TF + localization (shadow; joystick drives, no autonomy)
- `ros2 topic echo /joint_states` — 16 joints; RViz robot model matches the real pose (legs NOT out of phase → confirms the `LeftFront→fl` MotorStatus mapping; if a limb is wrong, fix `LEG_JOINTS` in `m20_udp_node.py`).
- Ground-plane check: in RViz the `/lidar/points` floor is flat at z≈0 → validates the `base_link→lidar_link` static TF (0.32028, 0, −0.013). Adjust the static TF (and re-derive GLIM `T_lidar_imu`) if the floor tilts or floats.
- Start nav_core; drive the robot with the handheld controller. AMCL should hold localization while walking (watch for the hemispherical-lidar near-field blind ring vs `range_min 0.9`, and body pitch swinging the p2l height band). No `/cmd_vel` is sent — nav_core is observed only.

### T4 — state services (robot SUSPENDED / hoisted, `enable_tx:=false`)
Verify each Type 2/Cmd 22 command before any ground motion:
- `ros2 service call /m20_udp_node/stand std_srvs/srv/Trigger` → robot stands (MotionParam 1).
- `.../sit` → sits (4); `.../estop` → soft e-stop (2) + `/robot/ready` goes false + latch; `.../reset` clears latch; `.../set_regular_mode` → telemetry `control_usage_mode` = 0.

### T5 — Cmd 21 calibration (open area, `enable_tx:=true`, controller in hand)
- Publish small steps to `/cmd_vel` (e.g. `linear.x` giving normalized 0.1 → 0.6); for each, record achieved speed from `/odom` and UDP `1002/4 LinearX/OmegaZ`. Fit `axis_scale_x/y/yaw` and `axis_deadband`; put them in `bridge_params.yaml`.
- Startup self-check: command 0.1 for 1 s, confirm achieved speed within 30 % of expected; abort if not.

### T6 — safety drills (open area, `enable_tx:=true`)
While the robot is moving under `/cmd_vel`:
- Stop publishing `/cmd_vel` → bridge streams zero within `watchdog_timeout` (0.5 s), robot stops.
- `docker kill m20_bridge` mid-motion; `docker kill nav_core` mid-motion; block port 30000 mid-motion — **record what the firmware does when the 20 Hz stream stops** (this behavior is unverified and load-bearing). Trigger the hardware e-stop; confirm recovery needs `~/reset`.

### GLIM mapping (before T-nav with a new site; needs only the controller)
1. Bring up `m20_bridge` (+ ad-hoc nothing — `/imu` and `/lidar/points` already flow). 
2. Record while walking the site with the controller (loops for closure):
   `ros2 bag record -o m20_site --compression-mode file --compression-format zstd /lidar/points /imu`
3. Offline on the workstation (GLIM built there): `ros2 launch rl_deploy glim_slam.launch.py` pointed at `config/glim/M20_REAL` + `ros2 bag play m20_site`. Sanity: map not smeared, loops close, `|/imu accel|≈9.8` at rest. If it bends on turns, tune `imu_time_offset` / `T_lidar_imu`.
4. `ply_to_2d_map.py` → `.pgm/.yaml`; SWAGGER (WS venv) → `graph.gml`. Copy both into GOS `~/maps`.

### T7 — end-to-end autonomy (`enable_tx:=true`)
`docker compose up`; RViz "2D Goal Pose" → route_server plans over `graph.gml` → orchestrator → DWB → `/cmd_vel` → bridge → robot walks the route. Watch DWB micro-commands vs the axis deadband near the goal (StoppedGoalChecker).

---

## Risks (carry into testing)
- **Cmd 21 never fired before** — validated only in T4–T6; no `/NAV_CMD` fallback is built into this image (add `drdds` + a NavCmd publisher if the UDP path fails).
- **Firmware behavior on stream/heartbeat loss is unverified** — T6 gates trust; until then treat the bridge as the only software stop besides the e-stop.
- **App/joystick contention** in Regular mode — disconnect them during autonomy; the node reports permission errors (57351) via telemetry.
- **Normalized scale is gait/speed-gear dependent** — LUT + startup self-check + `max_norm` cap.
- **OTA resets stock services and may rename `/IMU_YESENSE` / `/LIO_ODOM`** — T1 re-checks after any firmware update.
