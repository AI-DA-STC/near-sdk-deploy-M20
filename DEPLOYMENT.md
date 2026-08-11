# M20 deployment — robot-agnostic nav_core + M20 bridge

Two containers from **one image** on the GOS host (10.21.31.104). Nothing is
installed on AOS (10.21.31.103) — the bridge talks to stock firmware over the
UDP Inspection Protocol + direct DDS.

```
AOS 10.21.31.103 (stock, untouched)     ┌ m20_bridge container ─────────────┐  ┌ nav_core container ─────────────┐
 UDP :30000 ─1002/4 telemetry,estop───► │ m20_udp_node → /m20/telemetry      │  │ p2l /lidar/points → /scan       │
            ◄─heartbeat 1Hz / Cmd21 20Hz│   → /m20/telemetry, /robot/ready   │  │ amcl (map→odom), costmaps       │
 RTSP :8554 video1/2 ─────────────────► │ rtsp_camera1/2 → /camera1,/camera2 │  │ ekf /odom (+/imu) → odom→base   │
                                        │                                    │  │ route_server(graph)→orchestrator│
 GOS-local Foxy/Fast-DDS publishers     │ [Fast-DDS sidecar] /IMU →socket→   │─►│ nav2 → /cmd_vel                 │
  /IMU 200Hz ─────────────────────────► │   /IMU_fastdds → restamp → /imu    │◄─└─────────────────────────────────┘
  /LIDAR/POINTS 10Hz ~1.2MB ──────────► │ [Cyclone] restamp /LIDAR/POINTS    │
   (each needs a DIFFERENT rmw - below) │        → /lidar/points  (offset)   │
                                        │ static TF base_link→lidar_link     │
 laptop RViz (10.21.34.x) /goal_pose ──────────────────────────────────────────►  route_server
   (must also run rmw_cyclonedds_cpp)   │ RSP: M20 URDF → /robot_description │        (viz: /robot_description, TF)
                                        └────────────────────────────────────┘
```

### The two sensors need different middlewares

The M20 publishes `/LIDAR/POINTS` (~1.2 MB, 10 Hz) and `/IMU` (~300 B, 200 Hz)
from **Foxy over Fast-DDS**, and that is a manufacturer setting. Measured on the
robot from inside a Humble container:

| topic | CycloneDDS 0.10 | Fast-DDS 2.6 |
|---|---|---|
| `/LIDAR/POINTS` | **9.3 Hz, 10.8 MB/s** | matched **nothing** |
| `/IMU` | matched **nothing** | **200.0 Hz** |

Exactly complementary. `RMW_IMPLEMENTATION` is per-**process**, so no single
process can read both, and "matched nothing" is a *discovery-level* failure — not
degraded delivery. QoS and socket buffers were both tried and neither moves it.

So `m20_bridge` runs **CycloneDDS** as its primary RMW, keeping the lidar's
12 MB/s native, and `m20_bridge.launch.py` spawns one **Fast-DDS sidecar** that
carries `/IMU` across on a Unix datagram socket:

```
  imu_uds_source  [Fast-DDS]    /IMU  --raw CDR-->  /tmp/m20_imu_cdr.sock
  imu_uds_sink    [CycloneDDS]  socket --raw CDR-->  /IMU_fastdds
  restamp_imu     [CycloneDDS]  /IMU_fastdds -> /imu
```

`launch`'s `additional_env` gives that one process its own RMW inside the shared
container. `/IMU` is the stream that gets hopped, not the cloud, because it is
~60 kB/s against the cloud's ~12 MB/s. Measured through the hop: **200.0 Hz,
0 dropped**, with stamps and field values intact (raw CDR is copied, never
decoded, and both ends are Humble so the encoding is identical by construction).

Reproduce the table any time with `scripts/test_humble_ingest.sh`, which takes a
`TOPIC` override:

```bash
bash scripts/test_humble_ingest.sh              # /LIDAR/POINTS
TOPIC=/IMU bash scripts/test_humble_ingest.sh   # /IMU
```

**Do not "simplify" this onto one RMW** — whichever you choose, one sensor goes
silent and the LIO dies. `imu_via_fastdds:=false` drops the hop and subscribes
`/IMU` directly on the container's own RMW; correct only on a robot where that
RMW can actually see it.

Because a process gets one RMW, `nav_core` must match `m20_bridge`'s primary
(CycloneDDS). Nothing else is affected — the AOS link is raw UDP and the cameras
are RTSP over TCP. **Your laptop RViz must also**
`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

### Investigated and ruled out — please don't re-litigate

- **`net.core.rmem_max`.** A clamped socket buffer really does destroy large
  fragmented samples, and on a stock 208 KB kernel a 1.8 MB cloud managed only
  1–2 Hz of 10 over *both* middlewares. But this robot ships **562 MB**, so it
  was never the cause here. `scripts/preflight_gos.sh` still checks it as a guard.
- **Subscriber QoS.** The vendor publishes RELIABLE and the original
  subscription was best_effort, which matches but never re-requests a lost
  fragment. Plausible, and wrong: measured equivalent (9.33 vs 9.25 Hz on the
  lidar, 200 Hz either way on the IMU), and the failing cells fail identically
  under both settings. QoS is not what separates the two middlewares here.
- **A CycloneDDS relay in a Foxy sidecar.** Built and measured. CycloneDDS 0.7 —
  what Foxy ships — **segfaults in `rmw_create_node`** the instant it exchanges
  discovery with this robot's Fast-DDS participants, on domain 0, with any
  config including stock defaults. Configs that skip those participants' ports
  survive but then receive nothing, because those participants *are* the
  publishers. Surviving and receiving are mutually exclusive, so that approach
  cannot work. Removed.
- **Participant-index exhaustion** is real and still guarded against, see
  `MaxAutoParticipantIndex` in `config/cyclonedds_gos.xml`.

**Generic contract** (nav_core knows only these — port to another robot by
writing a new `<robot>_bridge` package + launch; nav_core is untouched):

| dir | topic | type | produced by |
|-----|-------|------|-------------|
| IN  | `/lidar/points` | PointCloud2 | bridge (restamp `/LIDAR/POINTS`) |
| IN  | `/imu`   | Imu      | bridge (Fast-DDS sidecar → `/IMU_fastdds` → restamp) |
| IN  | `/odom`  | Odometry | bridge (GOS-side LIO) |
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
| `/odom`         | **rf2o** scan-matching from `/scan` | **rf2o** scan-matching from `/scan` (same node — the onboard `/LIO_ODOM` is deliberately not used) |
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
- **Architecture check.** The GOS is arm64; build the image for it or it will not
  exec. `docker image inspect m20-deploy:humble --format '{{.Architecture}}'`
  must print `arm64`. From an x86 laptop, register QEMU first
  (`docker run --privileged --rm tonistiigi/binfmt --install arm64` — this resets
  on reboot), then
  `docker buildx build --platform linux/arm64 -f Dockerfile.deploy -t m20-deploy:humble --load .`
- If a build ever produces an image whose contents you do not recognise, rebuild
  with `--no-cache`; a stale buildx layer has served foreign files here before.

### T0.5 — host sanity (before any container)
- `./scripts/preflight_gos.sh` → `PREFLIGHT OK`. It confirms `net.core.rmem_max`
  is large and prints the **type and reliability** of `/LIDAR/POINTS` and `/IMU`
  as the robot advertises them.
- Not the historical failure cause on this robot (it ships 562 MB) but a cheap
  guard: a clamped buffer genuinely does destroy large fragmented samples, and
  `config/cyclonedds_gos.xml` asks CycloneDDS for 8 MB as a hard requirement.

### T1 — the sensor hop (robot powered, on stand)
Gate for everything DDS. In the `m20_bridge` container shell:

- `ros2 topic list` shows `/LIDAR/POINTS` and `/IMU`. **This only works under
  `rmw_cyclonedds_cpp`** — see the RMW section at the top of this file.
- `ros2 topic hz /LIDAR/POINTS` ≈ 10 Hz, `/IMU` ≈ 200 Hz.
- `ros2 topic hz /lidar/points /imu` ≈ the same rates, confirming the restamps.
- **The /IMU hop.** `/IMU` is read by a Fast-DDS sidecar, not by this container's
  CycloneDDS, so `ros2 topic hz /IMU` from a normal shell here shows **nothing** —
  that is expected, not a fault. Check the hop instead:
  ```bash
  ros2 topic hz /IMU_fastdds          # ≈200 Hz — the sidecar's output
  docker logs m20_bridge | grep cdr_uds_relay   # [source]/[sink] heartbeats
  ```
  Both heartbeats should report ~200 Hz with `0 dropped`. A `[source] 0 msg`
  means Fast-DDS is not seeing `/IMU`; a `[sink] 0 msg` with a healthy source
  means the socket path differs between the two processes.
- `ros2 topic info -v /LIDAR/POINTS /IMU` → both should read
  `RELIABLE`, matching `sensor_reliability:=reliable` in `docker-compose.yml`. A
  mismatch matches *nothing* and looks exactly like a transport fault;
  `restamp_relay` logs a loud `INCOMPATIBLE QoS` if it sees one.

**If `/imu` is silent:** the sidecar is the only thing that can read `/IMU`.
Confirm it is running (`docker logs m20_bridge | grep imu_uds`), that
`imu_via_fastdds` is `true`, and re-measure with
`TOPIC=/IMU bash scripts/test_humble_ingest.sh` — cells C/D (Fast-DDS) should
pass and A/B (CycloneDDS) should not. If C/D now fail too, the robot stopped
publishing.

**If `/lidar/points` is silent, in this order:**

1. **Confirm the RMW.** `docker exec m20_bridge printenv RMW_IMPLEMENTATION` must
   be `rmw_cyclonedds_cpp`. Under Fast-DDS this topic delivers **zero** samples
   on this robot. Run `scripts/test_humble_ingest.sh` to see all four
   RMW × QoS combinations measured side by side.
2. **`Failed to find a free participant index for domain 0`** (container exits at
   startup). Discovery is unicast, so each participant needs a free RTPS port
   slot, and `auto` only probes indices `0..MaxAutoParticipantIndex` — Cyclone's
   default is **9**, and the robot's own Foxy stack holds 0–9. The shipped config
   sets 60; raise it if this returns. Count what is taken with
   `ss -lun | awk '$4 ~ /:74[0-9][0-9]$/' | sort -u`.
   `ParticipantIndex=none` does *not* help — verified on CycloneDDS 0.7 and 0.10.
3. **Config typos.** Set `<Tracing><Verbosity>config</Verbosity>` in
   `config/cyclonedds_gos.xml`; Cyclone then echoes every parsed setting and
   flags bad ones. Note it rejects `KB` as a unit — use `B`, `MB` or `MiB`.
4. **A crash with no message.** Cyclone logs to stderr, which `ros2 launch`
   block-buffers, so a segfault loses the diagnosis. Point
   `<OutputFile>` at a file instead and read it after the process dies.

### T2 — bridge RX only (`enable_tx:=false`)
- `ros2 topic hz /imu /odom /lidar/points` match the source rates.
- `ros2 topic echo /imu --field header.stamp` ≈ `date +%s` (offset restamp working); consecutive `/imu` dt ≈ 5 ms (not bursty → GLIM-safe).
- `ros2 topic echo /m20/telemetry` shows MotionState/HES/Gait/Version; `/robot/ready` publishes.

### T3 — joints + TF + localization (shadow; joystick drives, no autonomy)
- `ros2 topic echo /joint_states` — 16 joints with position, velocity AND torque, sourced from `/JOINTS_DATA` by `m20_joints_node.py`. RViz's robot model must stand in the same posture as the real robot. If it renders inverted or splayed, the vendor→URDF conversion is wrong, not the transport: that node applies `q_urdf = wrap(q_vendor * JOINT_DIR + POS_OFFSET_RAD)`, the exact inverse of `gazebo_controller_ros2.py:342`. Both tables must stay in step with that file.
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
