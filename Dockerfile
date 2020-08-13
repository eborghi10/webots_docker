FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu16.04 as nvidia
FROM ubuntu:18.04
LABEL maintainer="Emiliano Borghi"

ARG uid
ENV USER webots

# Setup environment
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US.UTF-8
ENV \
  LANG=en_US.UTF-8 \
  DEBIAN_FRONTEND=noninteractive \
  TERM=xterm

# Dependencies
RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        apt-transport-https \
        apt-utils \
        bash-completion \
        build-essential \
        ca-certificates \
        gnupg2 \
        mesa-utils \
        software-properties-common \
        sudo \
        tmux \
        wget

# Create a user with passwordless sudo
USER root
RUN adduser --gecos "Development User" --disabled-password -u ${uid} $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Webots dependencies
RUN apt-get update && apt-get install -y \
  libusb-dev swig python2.7-dev libglu1-mesa-dev libglib2.0-dev \
  libfreeimage-dev libfreetype6-dev libxml2-dev libzzip-0-13 \
  libboost-dev libjpeg8-dev libpci-dev libgd-dev libtiff5-dev \
  libzip-dev python-pip libreadline-dev libassimp-dev libpng-dev \
  ffmpeg python3.6-dev python3.7-dev libxslt1-dev pbzip2 \
  xvfb lsb-release wget unzip zip libnss3 libnspr4 libxcomposite1 \
  libxcursor1 libxi6 libxrender1 libxss1 libasound2 libdbus-1-3 \
  xserver-xorg-video-dummy xpra xorg-dev libgl1-mesa-dev mesa-utils \
  libgl1-mesa-glx xvfb libxkbcommon-x11-dev

# Install Webots
WORKDIR /home/${USER}
RUN wget -qO- https://cyberbotics.com/Cyberbotics.asc | apt-key add -
RUN apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
RUN apt-get update && apt-get install -y webots

ENV WEBOTS_HOME=/usr/local/webots
# https://cyberbotics.com/doc/guide/running-extern-robot-controllers#running-extern-robot-controllers
ENV PYTHONPATH=${PYTHONPATH}:${WEBOTS_HOME}/lib/controller/python27
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${WEBOTS_HOME}/lib/controller

# Audio support
RUN usermod -aG audio ${USER}
RUN apt-get install -y alsa-utils

# Install ROS Melodic
# Setup sources.list for ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
# Setup keys for ROS
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# Install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
  python-rosdep \
  python-rosinstall
# Install ROS packages
RUN apt-get update && \
  apt-get install -y \
  ros-melodic-desktop
# Initialize rosdep
RUN rosdep init
USER ${USER}
RUN rosdep update
# Automatically source ROS workspace
RUN echo ". /opt/ros/melodic/setup.bash" >> /home/${USER}/.bashrc
ENV WS_DIR "/catkin_ws"
ENV CATKIN_SETUP_BASH "${WS_DIR}/devel/setup.bash"
RUN echo "[[ -f ${CATKIN_SETUP_BASH} ]] && . ${CATKIN_SETUP_BASH}" >> /home/${USER}/.bashrc

USER root

# Install dependencies from source
# http://wiki.ros.org/catkin/Tutorials/using_a_workspace#Installing_Packages
ENV DEPS_WS=/deps_ws
RUN mkdir -p ${DEPS_WS}/src
WORKDIR ${DEPS_WS}/src
RUN git clone https://github.com/eborghi10/moveit_grasps.git
RUN wstool init .
RUN wstool merge moveit_grasps/moveit_grasps.rosinstall
RUN wstool update
WORKDIR ${DEPS_WS}
USER ${USER}
RUN rosdep install --from-paths src --rosdistro=melodic -yi -r --os=ubuntu:bionic
USER root
RUN /bin/bash -c ". /opt/ros/melodic/setup.bash; \
        catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic; \
        cd build; make install"
RUN rm -r ${DEPS_WS}

# ROS extra packages
RUN apt-get update && apt-get install -y \
  ros-melodic-moveit \
  ros-melodic-moveit-resources \
  ros-melodic-position-controllers \
  ros-melodic-rgbd-launch \
  ros-melodic-webots-ros \
  # robotiq
  python-pymodbus \
  qtbase5-dev \
  ros-melodic-controller-manager \
  ros-melodic-effort-controllers \
  ros-melodic-gazebo-msgs \
  ros-melodic-gazebo-plugins \
  ros-melodic-gazebo-ros \
  ros-melodic-gazebo-ros-control \
  ros-melodic-hardware-interface \
  ros-melodic-joint-state-controller \
  ros-melodic-joint-trajectory-controller \
  ros-melodic-pcl-ros \
  ros-melodic-socketcan-interface \
  ros-melodic-soem

# Installing OpenGL for nvidia-docker2
# https://stackoverflow.com/a/53823600
COPY --from=nvidia /usr/local /usr/local
COPY --from=nvidia /etc/ld.so.conf.d/glvnd.conf /etc/ld.so.conf.d/glvnd.conf

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

COPY 10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

USER ${USER}
CMD /bin/bash
