# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Set environment variables for ROS 2
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

# Update and upgrade system packages
RUN apt-get update && apt-get upgrade -y

# Install locales
RUN apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Setup sources for ROS 2
RUN apt-get install -y software-properties-common curl
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-gazebo-ros-pkgs

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Install NVIDIA container toolkit dependencies
RUN apt-get update && apt-get install -y \
    cuda-keyring \
    cuda-drivers \
    cuda-toolkit-12-2 # Replace with appropriate CUDA version if needed for Isaac Sim

# Optional: Add NVIDIA Container Toolkit repository and install if not already done by CUDA drivers
# See https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
# for more details
# distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
#       && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
#       && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
#             sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
#             tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
# RUN apt-get update
# RUN apt-get install -y nvidia-container-toolkit
