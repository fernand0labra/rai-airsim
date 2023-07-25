#!/usr/bin/env python 

import math

import rospy
from sensor_msgs.msg import Image 
from sensor_msgs.msg import CameraInfo

import setup_path
import airsim

rospy.init_node("update_frame_id")
# client = airsim.MultirotorClient()

current_timestamp = None

scene_image = Image()
depth_image = Image()
camera_info = CameraInfo()

height = 320
width = 240
fov = 90

f = width /(2 * math.tan(fov * math.pi / 360))

Cu = width/2   # Horizontal Center
Cv = height/2  # Vertical Center

intrinsics_matrix = [f, 0, Cu,
                     0, f, Cv,
                     0, 0, 1 ]

projection_matrix = [1, 0, 0, -Cu,
                     0, 1, 0, -Cv,
                     0, 0, 0, f]

print("intrinsics_matrix -->", intrinsics_matrix)
print("projection_matrix -->", projection_matrix)

#Updating frame id for the error depth_front frame id does not match rgb_front frame id
class update_frame_id:
    def __init__(self):

        # Airsim ROS Wrapper
        self.sub_raw = rospy.Subscriber("/airsim_node/Drone_1/camera_1/Scene", Image, self.callback_raw)
        self.sub_depth = rospy.Subscriber("/airsim_node/Drone_1/camera_2/DepthPlanar", Image, self.callback_depth)
        self.sub_info = rospy.Subscriber("/airsim_node/Drone_1/camera_2/DepthPlanar/camera_info", CameraInfo, self.callback_info)

        # ethz-asl/unreal_airsim ROS package
        # self.sub_raw = rospy.Subscriber("/airsim_node/airsim_drone/Scene_cam/Scene", Image, self.callback_raw)
        # self.sub_depth = rospy.Subscriber("/airsim_node/airsim_drone/Depth_cam/DepthPlanar", Image, self.callback_depth)
        # self.sub_info = rospy.Subscriber("/airsim_node/airsim_drone/Depth_cam/DepthPlanar/camera_info", CameraInfo, self.callback_info)

        self.pub_depth = rospy.Publisher("/camera/depth", Image,  queue_size = 5)
        self.pub_raw = rospy.Publisher("/camera/rgb", Image, queue_size = 5)
        self.pub_info = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size = 5)

    def callback_raw(self, message):
        global current_timestamp
        current_timestamp = message.header.stamp

        global scene_image
        scene_image = message
        scene_image.header.frame_id = "world_enu"
        scene_image.header.stamp = current_timestamp

    def callback_info(self, message):
        global camera_info
        camera_info = message
        camera_info.header.frame_id = "world_enu"
        camera_info.header.stamp = current_timestamp

        # camera_info.K = intrinsics_matrix
        camera_info.P = projection_matrix

    def callback_depth(self, message):
        global depth_image
        depth_image = message
        depth_image.header.frame_id = "world_enu"
        depth_image.header.stamp = current_timestamp


update_frame_id = update_frame_id()

while not rospy.is_shutdown():
    update_frame_id.pub_depth.publish(depth_image)
    update_frame_id.pub_info.publish(camera_info)
    update_frame_id.pub_raw.publish(scene_image)
    rospy.sleep(1)

