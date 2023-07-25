#!/bin/bash

##############################################################
#                      Airsim ROS Wrapper                    #
##############################################################

# ---------------------------------------------------------#
# Single drone with monocular and depth cameras, and lidar #
# ---------------------------------------------------------#

source ./AirSim/ros/devel/setup.bash
roscd airsim_tutorial_pkgs
cp settings/front_stereo_and_center_mono.json ~/Documents/AirSim/settings.json

## Start your unreal package or binary here

roslaunch airsim_ros_pkgs airsim_node.launch;

# in a new pane / terminal
source ./AirSim/ros/devel/setup.bash
roslaunch airsim_tutorial_pkgs front_stereo_and_center_mono.launch


# ---------------------------------------------------------#
#         Two drones, with cameras, lidar, IMU each        #
# ---------------------------------------------------------#

source ./AirSim/ros/devel/setup.bash
roscd airsim_tutorial_pkgs
cp settings/two_drones_camera_lidar_imu.json ~/Documents/AirSim/settings.json

## Start your unreal package or binary here

roslaunch airsim_ros_pkgs airsim_node.launch;
roslaunch airsim_ros_pkgs rviz.launch


# ---------------------------------------------------------#
#          Twenty-five drones in a square pattern          #
# ---------------------------------------------------------#

source ./AirSim/ros/devel/setup.bash
roscd airsim_tutorial_pkgs
cp settings/twenty_five_drones.json ~/Documents/AirSim/settings.json

## Start your unreal package or binary here

roslaunch airsim_ros_pkgs airsim_node.launch;
roslaunch airsim_ros_pkgs rviz.launch