ARG ROS_DISTRO=jazzy

FROM ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full AS base

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-full python3-pip vim net-tools netcat-traditional tmux software-properties-common \
    ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-nav2-msgs ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-topic-tools ros-dev-tools \
    ros-${ROS_DISTRO}-clearpath-desktop ros-${ROS_DISTRO}-clearpath-simulator \
    ros-${ROS_DISTRO}-turtlebot4-simulator ros-${ROS_DISTRO}-irobot-create-nodes \
    python3-vcstool python3-apt

# for some reason MESA drivers are obsolete
RUN add-apt-repository ppa:kisak/kisak-mesa && \
    apt update && \
    apt upgrade -y

# install python requirements
COPY ./requirements.txt /requirements.txt

# create a virtual environment and install requirements
RUN python3 -m venv .venv && \
    . .venv/bin/activate && \
    pip install -r /requirements.txt

# create a directory for tmux configuration
# and enable mouse support
RUN mkdir -p /root/.config/tmux/ && \
    echo "set -g mouse on" >> /root/.config/tmux/tmux.conf

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=:2 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
  __GLX_VENDOR_LIBRARY_NAME=nvidia \
  __NV_PRIME_RENDER_OFFLOAD=1

# set root directory
WORKDIR /MRTP

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /.venv/bin/activate" >> /root/.bashrc
RUN echo "export PYTHONPATH=/usr/lib/python3/dist-packages:\$PYTHONPATH" >> /root/.bashrc

CMD ["/bin/bash"]