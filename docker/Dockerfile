FROM ros:humble-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# System dependencies
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    python3-vcstool git wget curl \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# MoveIt 2
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-planners \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    && rm -rf /var/lib/apt/lists/*

# RealSense SDK + ROS wrapper
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies (CPU-only for M1 Docker)
RUN pip3 install --no-cache-dir \
    ultralytics==8.1.0 \
    opencv-python-headless \
    numpy

# Visualization tools
FROM base AS viz
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# Development stage
FROM viz AS dev
WORKDIR /ros2_ws

COPY src/ src/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
