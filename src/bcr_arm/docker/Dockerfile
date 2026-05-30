FROM osrf/ros:humble-desktop-full-jammy

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

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

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

WORKDIR /workspaces/bcr_arm

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
COPY docker/setup_workspace.sh /usr/local/bin/bcr-setup-workspace

RUN chmod +x /ros_entrypoint.sh /usr/local/bin/bcr-setup-workspace

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
