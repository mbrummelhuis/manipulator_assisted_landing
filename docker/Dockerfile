# Use an official ROS 2 base image (e.g., Humble Hawksbill on Ubuntu 22.04)
# We use 
FROM ros:jazzy-ros-base

# Set environment variables
ENV ROS_DISTRO=jazzy
ENV ROS_DOMAIN_ID=75

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-argcomplete \
    bash-completion \
    ros-jazzy-ament-cmake-clang-format \
    git \
    nano \
    terminator \
    libboost-all-dev \
    python3-pip \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies (for TacTip driver)
RUN pip3 install -U --break-system-packages \
    numpy==1.26.4 \
    opencv-python==4.10.0.84 \
    scikit-image==0.24.0 \
    torch==2.5.1 \
    pytorch-model-summary==0.1.2 \
    vit-pytorch==1.8.8 \
    pandas==2.2.3 \
    matplotlib==3.9.2 \
    seaborn==0.13.2

WORKDIR /ros2_ws/aerial_tactile_servoing

