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
RUN sudo usermod --append --groups video $USERNAME

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
    cd ardupilot_gazebo && \
    git submodule update --init --recursive && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4

RUN sudo apt-get install ros-jazzy-mavros ros-jazzy-mavros-extras -l -y && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod a+x ./install_geographiclib_datasets.sh && \
    sudo ./install_geographiclib_datasets.sh

# TCP 5760 is what the sim exposes by default
EXPOSE 5760/tcp
EXPOSE 14550/udp
