#################################### Import base image
ARG IMAGE_NAME=autoware/autoware
ARG TAG_PREFIX=latest
ARG ROS_DISTRO=melodic
ARG VERSION=master

# cpu only
#FROM autoware/autoware:latest-melodic
#FROM autoware/autoware:local-melodic

# gpu
#FROM autoware/autoware:latest-melodic-cuda
#FROM autoware/autoware:local-melodic-cuda
#FROM $IMAGE_NAME:$TAG_PREFIX-$ROS_DISTRO-cuda
FROM $IMAGE_NAME:$TAG_PREFIX-$ROS_DISTRO

#################################### Set up "user" and environment variables
ARG USER=autoware
ENV USERNAME ${USER}
ARG USER_ID=1000
ARG GROUP_ID=15214

# Add user to video and audio group
RUN echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo,video,audio,dialout $USERNAME && \
        usermod  --uid $USER_ID $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME

USER ${USER}
ENV BUILD_HOME=/home/$USERNAME

ARG BUILD_HOME=$BUILD_HOME
ARG WORKSPACE_NAME="catkin_ws"

## Set up the shell
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

################################### Fix bugs
# Fix ROS GPG key expiration issue
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

## Fix Nvidia GPG key expiration issue
#RUN sudo rm /etc/apt/sources.list.d/cuda.list && sudo rm /etc/apt/sources.list.d/nvidia-ml.list && \
#    sudo apt-key del 7fa2af80 && \
#    mkdir -p /tmp/cuda_fix && cd /tmp/cuda_fix && \
#    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-keyring_1.0-1_all.deb && \
#    sudo dpkg -i cuda-keyring_1.0-1_all.deb

# Setup timezone
ENV TZ=America/New_York
RUN echo $TZ | sudo tee /etc/timezone
RUN sudo dpkg-reconfigure --frontend noninteractive tzdata

RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime
# RUN echo $TZ > /etc/timezone
RUN sudo apt-get clean && sudo apt-get update && sudo apt-get install -y locales
RUN sudo locale-gen en_US.UTF-8

################################### Install general packages (joystick, etc.)
# To fix any issues related to ROS apt repository key expiration, check (https://answers.ros.org/question/379190/apt-update-signatures-were-invalid-f42ed6fbab17c654/?answer=379194#post-id-379194)
RUN sudo apt-get update -y && sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-ackermann-* \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-hector-slam \
    ros-${ROS_DISTRO}-imu-tools \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rqt-robot-plugins \
    ros-${ROS_DISTRO}-video-stream-opencv \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-image-view \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-teb-local-planner \
    ros-${ROS_DISTRO}-stage-ros \
    ros-${ROS_DISTRO}-message-to-tf \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-autoware-msgs \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-can-msgs \
    ros-${ROS_DISTRO}-nmea-msgs \
    ros-${ROS_DISTRO}-nmea-navsat-driver \
    ros-${ROS_DISTRO}-sound-play \
    ros-${ROS_DISTRO}-jsk-visualization \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-can-msgs \
    ros-${ROS_DISTRO}-um7 \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-numpy \
    python3-opencv \
    python3-catkin-pkg-modules \
    python3-rospkg-modules \
    python3-empy \
    python-pip \
    python-wstool \
    python-setuptools \
    python-rosinstall \
    python-rosinstall-generator \
    libnlopt-dev \
    freeglut3-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libssh2-1-dev \
    libarmadillo-dev \
    libpcap-dev \
    libgl1-mesa-dev \
    python-bson \
    python-bson-ext \
    python-openssl \
    python-pam \
    python-twisted-bin \
    python-twisted-core \
    python-zope.interface \
    python3-apt \
    python3-pycurl \
    python3-gi \
    python-setuptools \
    python-pip \
    gir1.2-gtk-3.0 \
    python3-aptdaemon.gtk3widgets \
    linux-headers-generic \
    software-properties-common \
    ubuntu-drivers-common \
    libgtk2-perl dkms \
    dirmngr \
    gnupg2 \
    lsb-release \
    libprotobuf-dev \
    libleveldb-dev \
    libsnappy-dev \
    libopencv-dev \
    libhdf5-serial-dev \
    protobuf-compiler \
    libboost-all-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblmdb-dev \
    libatlas-base-dev \
    libopenblas-dev \
    iputils-ping \
    gedit \
    nautilus && \
    sudo rm -rf /var/lib/apt/lists/*

# Update pip and setuptools
RUN python3 -m pip install -U pip setuptools && python -m pip install -U pip setuptools

# Install python requirements
RUN python3 -m pip install \
    catkin_pkg \
    rospkg \
    matplotlib \
    pandas \
    scikit-learn \
    pyyaml

RUN python -m pip install \
    catkin_pkg \
    rospkg \
    matplotlib \
    pandas \
    scikit-learn \
    pyyaml

# Install Pygame
RUN python3 -m pip install -U pygame --user
RUN python -m pip install -U pygame --user

# Upgrade python2 packages
RUN python -m pip install --upgrade \
    scikit-image

# Upgrade python3 packages
RUN python3 -m pip install --upgrade \
    scikit-image



################################### Initialize directories
# Create ROS workspace. Todo: test with python3
RUN mkdir -p $BUILD_HOME/$WORKSPACE_NAME/src && cd $BUILD_HOME/$WORKSPACE_NAME && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python && source devel/setup.bash

# Create "Drivers" folder to compile drivers
RUN mkdir -p $BUILD_HOME/drivers

################################### Setup LIDAR
# Install Quanergy M8 LIDAR drivers
RUN mkdir -p $BUILD_HOME/drivers/Quanergy && cd $BUILD_HOME/drivers/Quanergy && \
    git clone https://github.com/privvyledge/quanergy_client.git && \
    cd quanergy_client && \
    mkdir build && cd build && \
    cmake .. && make && sudo make install

# Install Quanergy LIDAR ROS package. Todo: might need to embed sensors into one ros package
RUN cd $BUILD_HOME/$WORKSPACE_NAME/src && git clone https://github.com/privvyledge/quanergy_client_ros.git

################################### Setup GNSS. Todo

################################### Setup Cameras. Todo

################################### Setup Kvaser Drivers
# Cython must be installed as a seperate layer
RUN python -m pip install cython==0.25.2

# AutonomouStuff Packages and dependencies
RUN sudo apt-get update && sudo apt-get -y install software-properties-common apt-transport-https && sudo rm -rf /var/lib/apt/lists/*
RUN sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
RUN sudo sh -c 'echo "yaml https://s3.amazonaws.com/autonomoustuff-repo/autonomoustuff-public-$ROS_DISTRO.yaml $ROS_DISTRO" > /etc/ros/rosdep/sources.list.d/40-autonomoustuff-public-'$ROS_DISTRO'.list'

RUN sudo apt-add-repository ppa:astuff/kvaser-linux
RUN sudo apt-get update && sudo apt-get -y install ros-${ROS_DISTRO}-kvaser-interface && sudo rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update -qq && sudo apt-get install -y -q \
    kvaser-canlib-dev \
    kvaser-drivers-dkms \
    kvaser-linlib-dev \
    && sudo rm -rf /var/lib/apt/lists/*

################################### Setup New Eagle DBW stuff
RUN cd $BUILD_HOME/$WORKSPACE_NAME/src && git clone https://github.com/privvyledge/raptor-dbw-ros.git && \
    git clone https://github.com/privvyledge/raptor-dbw-rospy.git

################################### Vehicle Interface stuff (Autodriver package)
RUN cd $BUILD_HOME/$WORKSPACE_NAME/src && git clone https://github.com/privvyledge/autodriver.git

################################### Teleop stuff
RUN cd $BUILD_HOME/$WORKSPACE_NAME/src && git clone https://github.com/privvyledge/ros-ackermann-teleoperation.git

################################### Mark python scripts as executable
RUN cd $BUILD_HOME/$WORKSPACE_NAME/src/autodriver/nodes && chmod +x ./*
RUN cd $BUILD_HOME/$WORKSPACE_NAME/src/raptor-dbw-rospy/nodes && chmod +x ./*

################################### Visualization stuff. Todo: include different RVIZ config files.
# Todo: make my own RViz config files

################################### Autoware Launch. Todo: for now use autodriver package

################################### Setup vehicle simulation
# Fix gazebo GPU issues by updating and upgrading gazebo to the latest stable release
#RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main"
#RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
#    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
#    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743 && \
#    sudo apt update -y && \
#    sudo apt upgrade -y
#
## Enable motion simulation and traffic lights
#RUN mkdir -p $BUILD_HOME/Downloads/GazeboStuff && cd $BUILD_HOME/Downloads/GazeboStuff && \
#    git clone https://github.com/CPFL/osrf_citysim.git && \
#    cd osrf_citysim && mkdir build && cd build && cmake .. && sudo make install && \
#    cd ../worlds && erb simple_city.world.erb > simple_city.world

################################### Setup/Install Autoware
#RUN bash -c 'mkdir -p /home/$USERNAME/Autoware/src; \
#    cd /home/$USERNAME/Autoware; \
#    rm -rf install/ build/ log/; \
#    wget https://raw.githubusercontent.com/Autoware-AI/autoware.ai/$VERSION/autoware.ai.repos; \
#    vcs import src < autoware.ai.repos'
#
## Rebuild autoware
#RUN bash -c 'cd /home/$USERNAME/Autoware; \
#    source /opt/ros/$ROS_DISTRO/setup.bash; \
#    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'

# Add variables to /home/${USERNAME}/.bashrc and source files
RUN bash -c 'echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $BUILD_HOME/.bashrc; \
    echo "export $WORKSPACE_NAME=$BUILD_HOME/$WORKSPACE_NAME" >> $BUILD_HOME/.bashrc ; \
    echo "export PYTHONPATH=$BUILD_HOME/ssdcaffe/python:$PYTHONPATH" >> $BUILD_HOME/.bashrc; \
    echo "source $BUILD_HOME/$WORKSPACE_NAME/devel/setup.bash" >> $BUILD_HOME/.bashrc; \
    echo "source /usr/local/share/citysim/setup.sh" >> $BUILD_HOME/.bashrc; \
    echo "source $BUILD_HOME/Autoware/install/setup.bash" >> $BUILD_HOME/.bashrc; \
    echo "export QT_X11_NO_MITSHM=1" >> /home/$USERNAME/.bashrc; \
    source $BUILD_HOME/.bashrc'

# Setup permissions
USER root
ENTRYPOINT ["/tmp/entrypoint.sh"]
