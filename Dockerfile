FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Basic ROS 2 setup in .bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Install essential tools and ROS 2 navigation/SLAM packages
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    tmux \
    ruby \
    ruby-dev \
    build-essential \
    wget \
    vim \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-behavior-tree \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav2-util \
    ros-jazzy-nav2-rviz-plugins \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-gz-ros2-control ros-jazzy-gz-ros2-control-demos \
    ros-jazzy-topic-tools \
    ros-jazzy-ros-gz ros-jazzy-gz-plugin-vendor ros-jazzy-gz-sim-vendor \
    ros-jazzy-twist-mux \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ackermann-steering-controller \
    ros-jazzy-cv-bridge \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install tmuxinator gem
RUN gem install tmuxinator

# Create ROS 2 workspace and clone robobase package
RUN mkdir -p /root/robows/src && \
    cd /root/robows/src && \
#    git clone https://github.com/sim-daas/robobase && \
    cd /root/robows && \
    colcon build --symlink-install && \
    echo "source /root/robows/install/setup.bash" >> /root/.bashrc

# Set working directory
WORKDIR /root/

# Default command: start a shell
CMD ["/bin/bash"]
