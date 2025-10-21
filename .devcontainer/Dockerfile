FROM osrf/ros:jazzy-desktop-full

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME
ENV USER=$USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video render $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

# Source the workspace setup file
RUN echo "source /ros2_tutorials/install/setup.bash" >> ~/.bashrc

# Install additional ROS packages
RUN sudo apt-get install -y python3-pip

RUN sudo apt-get update && \
    sudo apt install -y qt6-base-dev

RUN sudo apt-get install -y \
    ros-${ROS_DISTRO}-image-geometry \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-image-pipeline

WORKDIR /home/$USERNAME
# Install ArduPilot
RUN git clone https://github.com/ArduPilot/ardupilot.git ardupilot && \
    cd ardupilot && \
    Tools/environment_install/install-prereqs-ubuntu.sh -y && \
    . ~/.profile && \
    git submodule update --init --recursive && \
    ./waf configure --board sitl --debug && \
    ./waf copter


# Install Ardupilot_GAZEBO
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git ardupilot_gazebo && \
    sudo apt update && \
    sudo apt install -y wget lsb-release gnupg curl && \
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    sudo apt update && \
    sudo apt install -y gz-harmonic libgz-sim8-dev rapidjson-dev && \
    sudo apt install -y libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl && \
    export GZ_VERSION=harmonic && \
    cd ardupilot_gazebo && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j4 && \
    echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc && \
    echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc && \
    echo 'export GZ_SIM_RESOURCE_PATH=$/ros2_tutorials/new_models:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

RUN sudo apt-get install ros-jazzy-mavros ros-jazzy-mavros-extras -y && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod a+x ./install_geographiclib_datasets.sh && \
    sudo ./install_geographiclib_datasets.sh

# TCP 5760 is what the sim exposes by default
EXPOSE 5760/tcp
EXPOSE 14550/udp

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


RUN sudo apt-get install -y nautilus gedit