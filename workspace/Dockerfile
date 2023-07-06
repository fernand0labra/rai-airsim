FROM ghcr.io/epicgames/unreal-engine:dev-5.0

RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
    wget
    # gpg-agent

# RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# RUN sudo apt-get update && sudo apt-get install -q -y --no-install-recommends \
#     ros-noetic-desktop \
#     # Building Dependencies
#     python3-rosdep \
#     python3-rosinstall \
#     python3-rosinstall-generator \
#     python3-wstool \
#     build-essential

# ROS Noetic - Ubuntu 20.02
RUN wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh && chmod +x ./ros_install_noetic.sh && ./ros_install_noetic.sh

RUN git clone https://github.com/fernand0labra/ros-airsim-compatibility && \
    cd ros-airsim-compatibility/workspace/src && \
    git clone https://github.com/ethz-asl/AirSim.git && \
    export AIRSIM_PATH=/ros-airsim-compatibility/workspace/src/AirSim && \
    cd AirSim && \
    chmod u+x setup.sh && ./setup.sh && \
    chmod u+x build.sh && ./build.sh
    
