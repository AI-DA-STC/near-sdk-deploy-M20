[简体中文](./README.md)

[![Discord](https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white)](https://discord.gg/gdM9mQutC8)

## Sim-to-sim

```bash
pip install "numpy < 2.0" mujoco
git clone https://github.com/DeepRoboticsLab/M20_rl_deploy.git

# Compile
cd M20_rl_deploy
source /opt/ros/<ros-distro>/setup.bash
colcon build --packages-up-to rl_deploy --cmake-args -DBUILD_PLATFORM=x86
```

```bash
# Run (Open 2 terminals)
# Terminal 1
source install/setup.bash
ros2 run rl_deploy rl_deploy

# Terminal 2 
python3 src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py
```

### Control (Terminal 2)

tips：right click simulator window and select "always on top"

- z： default position
- c： rl control default position
- wasd：forward/leftward/backward/rightward
- qe：clockwise/counter clockwise


# Sim-to-Real
This process is almost identical to simulation-simulation. You only need to add the step of connecting to Wi-Fi to transfer data, and then modify the compilation instructions. The default control mode is currently set to keyboard mode. We will be adding controller support in future updates. Stay tuned.


Download drdds-ros2-msgs packages from [here](https://drive.google.com/file/d/1Nvxot_LOKMvLAr608kFtZ28gfsgA_dsx/view?usp=sharing) and install

```bash

# computer and gamepad should both connect to WiFi
# WiFi: M20********
# Passward: 12345678 (If wrong, contact technical support)

# scp to transfer files to quadruped (open a terminal on your local computer) password is ' (a single quote)
scp -r ~/M20_rl_deploy user@10.21.31.103:~/

# ssh connect for remote development, 
ssh user@10.21.31.103
cd M20_rl_deploy
colcon build --packages-up-to rl_deploy --cmake-args -DBUILD_PLATFORM=arm


sudo su # Root
sudo dpkg -i drdds-ros2-msgs.v1.0.2+.arm64.2510291519.deb
source /opt/ros/foxy/setup.bash #source ROS2 env
source /opt/robot/scripts/setup_ros2.sh
ros2 service call /SDK_MODE drdds/srv/StdSrvInt32 command:\ 200 # /200 is /JOINTS_DATA topic frequency, recommended below 500 Hz

# Run
source install/setup.bash
ros2 run rl_deploy rl_deploy

# exit sdk mode：
ros2 service call /SDK_MODE drdds/srv/StdSrvInt32 command:\ 0

# keyboard control
- z： default position
- c： rl control default position
- wasd：forward/leftward/backward/rightward
- qe：clockwise/counter clockwise
```

