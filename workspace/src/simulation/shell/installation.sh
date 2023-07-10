#!/bin/bash

# Unreal Engine
git clone -b 4.27 https://github.com/EpicGames/UnrealEngine.git
chmod u+x Setup.sh; ./Setup.sh
chmod u+x GenerateProjectFiles.sh; ./GenerateProjectFiles.sh
make

./Engine/Binaries/Linux/UE4Editor


# AirSim Microsoft Fork
export AIRSIM_PATH_1=/media/student/New\\ 
export AIRSIM_PATH_2=\ Volume/fernand0labra/ros-airsim-compatibility/workspace/src/AirSim
export AIRSIM_PATH=$AIRSIM_PATH_1$AIRSIM_PATH_2 # Set the target destination.

cd $AIRSIM_PATH

sudo apt-get install libboost-all-dev

git clone https://github.com/ethz-asl/AirSim.git
cd AirSim
chmod u+x setup.sh; ./setup.sh 
chmod u+x build.sh; ./build.sh

./Engine/Binaries/Linux/UE4Editor $AIRSIM_PATH/Unreal/Environments/Blocks/Blocks.uproject


# Unreal Airsim ROS Package
sudo apt-get install python3-wstool python3-catkin-tools ros-noetic-cmake-modules ros-noetic-tf2-sensor-msgs
cd ..

git clone https://github.com/ethz-asl/unreal_airsim.git

# wstool init . .rosinstall  # If non existant
wstool update

cd ~/ros-airsim-compatibility/workspace/src/unreal_airsim/
echo "set(AIRSIM_ROOT $AIRSIM_PATH)" > ./AirsimPath.txt
catkin build unreal_airsim
source ../devel/setup.bash

# With specific configuration
# roslaunch unreal_airsim parse_config_to_airsim.launch source:=path/to/my_settings.yaml

# With generic configuration
roslaunch unreal_airsim parse_config_to_airsim.launch 
roslaunch unreal_airsim demo.launch