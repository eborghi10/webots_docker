ARG UBUNTU_VERSION=18.04
# https://hub.docker.com/r/nvidia/cudagl
ARG ARCH=gl
ARG CUDA=10.2
FROM nvidia/cuda${ARCH}:${CUDA}-base-ubuntu${UBUNTU_VERSION} as base

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
        cmake \
        git \
        gnupg2 \
        libxt-dev \
        mesa-utils \
        software-properties-common \
        sudo \
        tmux \
        unzip \
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
RUN git clone https://github.com/eborghi10/moveit_grasps.git -b melodic-devel
RUN wstool init .
RUN wstool merge moveit_grasps/moveit_grasps.rosinstall
RUN wstool update
WORKDIR ${DEPS_WS}
USER ${USER}
RUN rosdep install --from-paths src --rosdistro=melodic -yi -r --os=ubuntu:bionic
USER root
RUN /bin/bash -c ". /opt/ros/melodic/setup.bash; \
        catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic -DCMAKE_BUILD_TYPE=Release; \
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
  ros-melodic-tf2-sensor-msgs \
  ros-melodic-socketcan-interface \
  ros-melodic-soem

######################################################

# # Install Eigen
# RUN cd /opt \
#     && git clone https://github.com/eigenteam/eigen-git-mirror eigen \
#     && cd eigen \
#     && git checkout tags/3.2.0 \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make -j$(nproc) \
#     && make install

# # Install VTK
# RUN cd /opt \
#     && git clone https://github.com/Kitware/VTK VTK \
#     && cd VTK \
#     && git checkout tags/v8.0.0 \
#     && mkdir build \
#     && cd build \
#     && cmake -DCMAKE_BUILD_TYPE:STRING=Release -D VTK_RENDERING_BACKEND=OpenGL .. \
#     && make -j$(nproc) \
#     && make install

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    libboost-all-dev \
    libflann-dev

# Install PCL
RUN cd /opt \
    && wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.11.0.zip \
    && unzip pcl-1.11.0.zip \
    && cd pcl-pcl-1.11.0 \
    && mkdir build \
    && cd build \
    && cmake -D CMAKE_BUILD_TYPE=Release -D BUILD_GPU=ON .. \
    && make -j$(nproc) \
    && make install

# Install Opencv
RUN cd /opt \
    && wget https://github.com/opencv/opencv/archive/3.4.3.zip \
    && unzip 3.4.3.zip \
    && cd opencv-3.4.3 \
    && mkdir build \
    && cd build \
    && cmake -D WITH_OPENMP=ON \
             -D ENABLE_PRECOMPILED_HEADERS=OFF \
             -D WITH_CUDA=ON \
             -D WITH_OPENGL=ON \..\
    && make -j$(nproc) \
    && make install

# Install GPD
RUN cd /opt \
    && git clone https://github.com/eborghi10/gpd gpd \
    && cd gpd \
    && mkdir build \
    && cd build \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local .. \
    && make -j$(nproc) \
    && make install

# Install Deep Grasp Demo
ENV GRASP_WS=/grasp_ws
RUN mkdir -p ${GRASP_WS}/src
WORKDIR ${GRASP_WS}/src
RUN echo ""
RUN git clone https://github.com/eborghi10/deep_grasp_demo.git
RUN wstool init .
RUN wstool merge deep_grasp_demo/.rosinstall
RUN wstool update
WORKDIR ${GRASP_WS}
USER ${USER}
RUN rosdep install --from-paths src --rosdistro=melodic -yi -r --os=ubuntu:bionic
USER root
RUN /bin/bash -c ". /opt/ros/melodic/setup.bash; \
        catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic -DCMAKE_BUILD_TYPE=Release; \
        cd build; make install"
RUN rm -r ${GRASP_WS}

######################################################

COPY params/* /GPD/gpd/models/lenet/15channels/params/

######################################################

# Webots config
RUN mkdir -p /home/${USER}/.config/Cyberbotics
COPY --chown=${USER}:${USER} Webots-R2020b.conf /home/${USER}/.config/Cyberbotics/Webots-R2020b.conf
RUN chown -R ${USER}:${USER} /home/${USER}/.config/Cyberbotics

USER ${USER}
CMD /bin/bash
