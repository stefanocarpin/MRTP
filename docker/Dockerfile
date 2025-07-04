ARG ROS_DISTRO=jazzy

FROM ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full AS base

# any utilities you want
RUN apt-get update && apt-get install -y git wget python3-full python3-pip vim net-tools netcat-traditional tmux \
    ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-nav2-msgs ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-topic-tools

# install python requirements
COPY ../requirements.txt /requirements.txt

# create a virtual environment and install requirements
RUN python3 -m venv .venv && \
    . .venv/bin/activate && \
    pip install -r /requirements.txt

# create a directory for tmux configuration
# and enable mouse support
RUN mkdir -p /root/.config/tmux/ && \
    echo "set -g mouse on" >> /root/.config/tmux/tmux.conf

# configure DISPLAY env variable for novnc connection
ENV DISPLAY=:2

# set root directory
WORKDIR /MRTP

# clone the MRTP repository
RUN git clone https://github.com/stefanocarpin/MRTP

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /.venv/bin/activate" >> /root/.bashrc
RUN echo "export PYTHONPATH=/usr/lib/python3/dist-packages:\$PYTHONPATH" >> /root/.bashrc

CMD ["/bin/bash"]


FROM base AS husky
ARG CLEARPATH=/root/clearpath
ARG CLEARPATH_WS=/root/clearpath_ws

# any utilities you want
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-clearpath-desktop ros-${ROS_DISTRO}-clearpath-simulator python3-vcstool python3-apt

RUN mkdir ${CLEARPATH} -p

COPY ../MRTP/src/gazeboenvs/models/husky/robot.yaml ${CLEARPATH}/robot.yaml

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    ros2 run clearpath_generator_common generate_bash -s ${CLEARPATH}

# Clearpath Husky simulation dependencies
RUN mkdir ${CLEARPATH_WS}/src -p && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${CLEARPATH_WS} && \
    wget https://raw.githubusercontent.com/clearpathrobotics/clearpath_simulator/${ROS_DISTRO}/dependencies.repos && \
    vcs import src < dependencies.repos && \
    rosdep install -r --from-paths src -i -y && \
    colcon build --symlink-install

RUN echo "source ${CLEARPATH}/setup.bash" >> /root/.bashrc
RUN echo "source ${CLEARPATH_WS}/install/setup.bash" >> /root/.bashrc


FROM base AS turtlebot

RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-turtlebot4-simulator
