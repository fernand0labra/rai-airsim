#!/bin/bash

##############################################################
#                        Unreal Engine                       #
##############################################################

git clone -b 4.27 https://github.com/EpicGames/UnrealEngine.git
chmod u+x Setup.sh; ./Setup.sh
chmod u+x GenerateProjectFiles.sh; ./GenerateProjectFiles.sh
make

./UnrealEngine-4.27/Engine/Binaries/Linux/UE4Editor


##############################################################
#                    AirSim Microsoft Fork                   #
##############################################################

export AIRSIM_PATH_1=/media/student/New\\ 
export AIRSIM_PATH_2=\ Volume/AirSim
export AIRSIM_PATH=$AIRSIM_PATH_1$AIRSIM_PATH_2 # Set the target destination.

cd $AIRSIM_PATH

sudo apt-get install libboost-all-dev

git clone https://github.com/ethz-asl/AirSim.git
cd AirSim
chmod u+x setup.sh; ./setup.sh 
chmod u+x build.sh; ./build.sh

# UnrealEngine & AirSim expected to be in the same folder
./UnrealEngine-4.27/Engine/Binaries/Linux/UE4Editor ../../../../AirSim/Unreal/Environments/Blocks/Blocks.uproject
./UnrealEngine-4.27/Engine/Binaries/Linux/UE4Editor ~/Documents/Unreal\ Projects/Architecture/Architecture.uproject

#------------------------------------------------------------#
#             Python API (AirSim/PythonClient)               #
#------------------------------------------------------------#

pip install msgpack-rpc-python
cd AirSim/PythonClient/car; python3 hello_car.py
cd AirSim/PythonClient/multirotor; python3 hello_drone.py  

#------------------------------------------------------------#
#              Airsim ROS Wrapper (AirSim/ros)               #
#------------------------------------------------------------#

sudo apt-get install ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-mavros*
pip install "git+https://github.com/catkin/catkin_tools.git#egg=catkin_tools"  # Ubuntu 20.02

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

cd AirSim/ros; catkin build;

source devel/setup.bash;
roslaunch airsim_ros_pkgs airsim_node.launch;
roslaunch airsim_ros_pkgs rviz.launch;


##############################################################
#                   Unreal Airsim ROS Package                #
##############################################################

# Mesh Position
sudo apt-get install python3-wstool python3-catkin-tools ros-noetic-cmake-modules ros-noetic-tf2-sensor-msgs
git clone https://github.com/ethz-asl/unreal_airsim.git
git clone https://github.com/ethz-asl/voxblox.git
git clone https://github.com/ethz-asl/voxgraph.git
git clone https://github.com/ethz-asl/cblox
git clone https://github.com/ethz-asl/protobuf_catkin
git clone https://github.com/ethz-asl/ceres_catkin

wstool init . .rosinstall  # If non existant
wstool update

cd ~/ros-airsim-compatibility/workspace/src/unreal_airsim/
echo "set(AIRSIM_ROOT $AIRSIM_PATH)" > ./AirsimPath.txt
catkin build unreal_airsim
source ./fernand0labra/ros-airsim-compatibility/workspace/devel/setup.bash

sudo apt-get install python-is-python3
pip3 install --upgrade scipy

# With specific configuration
# roslaunch unreal_airsim parse_config_to_airsim.launch source:=path/to/my_settings.yaml

# With generic configuration
roslaunch unreal_airsim parse_config_to_airsim.launch source:=./src/simulation/settings/unreal_airsim_drone.yaml
roslaunch unreal_airsim demo.launch