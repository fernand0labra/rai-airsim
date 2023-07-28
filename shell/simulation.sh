#!/bin/bash

##############################################################
#                        Unreal Engine                       #
##############################################################

# Start Editor
./UnrealEngine-4.27/Engine/Binaries/Linux/UE4Editor

# Start .uproject -> UnrealEngine & AirSim expected to be in the same folder
./UnrealEngine-4.27/Engine/Binaries/Linux/UE4Editor ../../../../AirSim/Unreal/Environments/Blocks/Blocks.uproject
./UnrealEngine-4.27/Engine/Binaries/Linux/UE4Editor ../../../../AirSim/Unreal/Environments/Matterport/Blocks.uproject


##############################################################
#                    AirSim Microsoft Fork                   #
##############################################################

#------------------------------------------------------------#
#             Python API (AirSim/PythonClient)               #
#------------------------------------------------------------#

cd AirSim/PythonClient/car; python3 hello_car.py
cd AirSim/PythonClient/multirotor; python3 hello_drone.py  

#------------------------------------------------------------#
#              Airsim ROS Wrapper (AirSim/ros)               #
#------------------------------------------------------------#

source devel/setup.bash;
roslaunch airsim_ros_pkgs airsim_node.launch;
roslaunch airsim_ros_pkgs rviz.launch;


##############################################################
#                   Unreal Airsim ROS Package                #
##############################################################

# With specific configuration
# roslaunch unreal_airsim parse_config_to_airsim.launch source:=path/to/my_settings.yaml

# With generic configuration
roslaunch unreal_airsim parse_config_to_airsim.launch source:=./src/simulation/settings/unreal_airsim_drone.yaml
roslaunch unreal_airsim demo.launch