#!/usr/bin/env python

from sensor_msgs.msg import Image
import numpy as np
import rospy

import matplotlib.pyplot as plt

def depth_conversion(point_depth, f):
    H = point_depth.shape[0]
    W = point_depth.shape[1]
    i_c = np.float32(H) / 2 - 1
    j_c = np.float32(W) / 2 - 1
    columns, rows = np.meshgrid(np.linspace(0, W-1, num=W, dtype=np.float32), np.linspace(0, H-1, num=H, dtype=np.float32))
    distance_from_center = ((rows - i_c)**2 + (columns - j_c)**2)**(0.5)
    plane_depth = point_depth / (1 + (distance_from_center / f)**2)**(0.5)
    return plane_depth

def callback(image: Image):

    byte_array_float32 = np.frombuffer(np.array(image.data), dtype=np.float32)
    point_depth_float64 = byte_array_float32.reshape(image.height, image.width)
    point_depth_converted_float32 = depth_conversion(point_depth_float64, point_depth_float64.shape[1] / 2)
    point_depth_converted_uint8 = point_depth_converted_float32.reshape(image.height * image.width).view(np.uint8).astype(np.uint8)

    depth_to_plane_msg.header = image.header
    depth_to_plane_msg.encoding = image.encoding
    depth_to_plane_msg.height = image.height
    depth_to_plane_msg.width = image.width
    depth_to_plane_msg.step = image.step
    depth_to_plane_msg.data = point_depth_converted_uint8.tolist()


rospy.init_node('ros_depth_handler', anonymous=True)
rospy.Subscriber('/airsim_drone/Depth_cam', Image, callback) 

depth_to_plane_msg = Image()
pub_depth_cam_plane = rospy.Publisher('/airsim_drone/Depth_cam_plane', Image, queue_size=10)

while not rospy.is_shutdown():
    pub_depth_cam_plane.publish(depth_to_plane_msg)
    rospy.sleep(1) 