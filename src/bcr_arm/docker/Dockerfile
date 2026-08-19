FROM osrf/ros:humble-desktop-full-jammy AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV AMENT_TRACE_SETUP_FILES=
ENV AMENT_PYTHON_EXECUTABLE=/usr/bin/python3


RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-numpy \
    python3-pandas \
    python3-pip \
    gazebo \
    libgazebo-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-gz-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-moveit \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-visual-tools \
    ros-humble-nav2-msgs \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-ros-gz-sim \
    ros-humble-sensor-msgs-py \
    ros-humble-tf-transformations \
    ros-humble-urdf-tutorial \
    ros-humble-dynamixel-sdk \
    usbutils \
    udev \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

# --- Perception additions
FROM base AS perception

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-vcstool \
    git \
    wget \
    libgl1 \
    libgomp1 \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-message-filters \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rviz2 \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    ultralytics==8.4.67 \
    opencv-python-headless \
    open3d \
    "numpy<2" \
    setuptools==58.2.0

# --- Final dev image
FROM perception AS dev

WORKDIR /workspaces/bcr_arm

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
COPY docker/setup_workspace.sh /usr/local/bin/bcr-setup-workspace

RUN chmod +x /ros_entrypoint.sh /usr/local/bin/bcr-setup-workspace

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
