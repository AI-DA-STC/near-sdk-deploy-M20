FROM osrf/ros:humble-desktop-full

SHELL [ "/bin/bash", "-c" ]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        wget \
        git \
        nano \
        build-essential \
        cmake \
        python3-pip \
        python3-venv \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        libgl1-mesa-glx \
        libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge \
        ros-humble-diagnostic-updater \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
    && rm -rf /var/lib/apt/lists/*
# -----------------------------------------------------------------------------

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "[ -f /root/ros_ws/install/setup.bash ] && source /root/ros_ws/install/setup.bash" >> /root/.bashrc
