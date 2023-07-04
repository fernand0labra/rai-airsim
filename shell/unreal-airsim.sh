#!/bin/bash

# Unreal Engine
git clone https://github.com/EpicGames/UnrealEngine.git
chmod u+x Setup.sh; ./Setup.sh
chmod u+x GenerateProjectFiles.sh; ./GenerateProjectFiles.sh
make

./Engine/Binaries/Linux/UE4Editor


# AirSim Microsoft Fork
export AIRSIM_PATH=/home/ltu/Desktop/ros-airsim-compatibility/workspace/src/AirSim # Set the target destination.
cd $AIRSIM_PATH
git clone https://github.com/ethz-asl/AirSim.git
cd Airsim
chmod u+x setup.sh; ./setup.sh 
chmod u+x build.sh; ./build.sh

./Engine/Binaries/Linux/UE4Editor $AIRSIM_PATH/Unreal/Environments/Blocks/Blocks.uproject


# Unreal Airsim ROS Package
sudo apt-get install python3-wstool python3-catkin-tools ros-noetic-cmake-modules ros-noetic-tf2-sensor-msgs
cd ..

git clone -b 4.25.4 https://github.com/ethz-asl/unreal_airsim.git

wstool init . ./src/.rosinstall
wstool update

echo "set(AIRSIM_ROOT $AIRSIM_PATH)" > ./AirsimPath.txt
catkin build unreal_airsim
source ../devel/setup.bash

roslaunch unreal_airsim parse_config_to_airsim.launch 
roslaunch unreal_airsim demo.launch