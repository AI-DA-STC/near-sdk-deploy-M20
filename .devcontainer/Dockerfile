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

# ---- GLIM SLAM (offline mapping, koide3 PPA) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        gpg \
    && rm -rf /var/lib/apt/lists/*

RUN curl -s https://koide3.github.io/ppa/setup_ppa.sh | bash

# koide3's PPA packages link against CUDA but don't ship its runtime — add
# NVIDIA's own network repo to supply it.
RUN wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && rm -f cuda-keyring_1.1-1_all.deb

# cuda12.6, not 12.2/13.x: matches the workstation host's own already-working
# install (confirmed via dpkg + ldd on the host) — an RTX 5090/Blackwell GPU,
# so this is empirical, not just version-compatibility guesswork. ldd there
# shows the entire CUDA dependency is libcudart.so.12, hence cuda-cudart-12-6
# alone (no cuBLAS/cuSOLVER, no full cuda-toolkit/nvcc needed).
RUN apt-get update && apt-get install -y --no-install-recommends \
        libiridescence-dev \
        libboost-all-dev \
        libglfw3-dev \
        libmetis-dev \
        cuda-cudart-12-6 \
        libgtsam-points-cuda12.6-dev \
        ros-humble-glim-ros-cuda12.6 \
        zenity \
    && rm -rf /var/lib/apt/lists/* \
    && ldconfig
# -----------------------------------------------------------------------------

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "[ -f /root/ros_ws/install/setup.bash ] && source /root/ros_ws/install/setup.bash" >> /root/.bashrc
