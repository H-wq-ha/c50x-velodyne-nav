FROM ros:noetic-ros-base-focal
SHELL ["/bin/bash", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake libgoogle-glog-dev libgflags-dev libsuitesparse-dev \
    libglew-dev libeigen3-dev libyaml-cpp-dev libpcap-dev libopencv-dev \
    ros-noetic-velodyne-msgs ros-noetic-pcl-ros ros-noetic-eigen-conversions \
    ros-noetic-tf-conversions ros-noetic-angles ros-noetic-diagnostic-updater \
    ros-noetic-cv-bridge && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
