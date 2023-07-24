import numpy as np
import rospy
import math

import cv2
from sensor_msgs.msg import Image, PointCloud2
import matplotlib.pyplot as plt

import setup_path
import airsim

color = (0,255,0)
rgb = "%d %d %d" % color

def savePointCloud(image, fileName):
   f = open(fileName, "w")
   for x in range(image.shape[0]):
     for y in range(image.shape[1]):
        pt = image[x,y]
        if (math.isinf(pt[0]) or math.isnan(pt[0])):
          # skip it
          None
        else: 
          f.write("%f %f %f %s\n" % (pt[0], pt[1], pt[2]-1, rgb))
   f.close()

def depth_conversion(point_depth, f):
    H = point_depth.shape[0]
    W = point_depth.shape[1]
    i_c = np.float32(H) / 2 - 1
    j_c = np.float32(W) / 2 - 1
    columns, rows = np.meshgrid(np.linspace(0, W-1, num=W, dtype=np.float32), np.linspace(0, H-1, num=H, dtype=np.float32))
    distance_from_center = ((rows - i_c)**2 + (columns - j_c)**2)**(0.5)
    print(distance_from_center, f)
    plane_depth = point_depth / (1 + (distance_from_center / f)**2)**(0.5)
    return plane_depth

def callback(image: airsim.ImageResponse):

    # byte_array_float64 = np.frombuffer(np.array(image.image_data_float), dtype=np.float64)
    # byte_array_uint8 = byte_array_float64.view(np.uint8).astype(np.uint8)

    # point_depth_float64 = byte_array_float64.reshape(image.height, image.width)
    # point_depth_uint8 = point_depth_float64.view(np.uint8).astype(np.uint8)

    # point_depth_converted_float64 = depth_conversion(point_depth_float64, point_depth_float64.shape[1] / 2)
    # point_depth_converted_uint8 = point_depth_converted_float64.view(np.uint8).astype(np.uint8)

    byte_array_uint8 = cv2.imdecode(np.frombuffer(bytearray(image.image_data_uint8), np.uint8) , cv2.IMREAD_UNCHANGED)
    # point_depth_float32 = byte_array_uint8.view(np.float32).astype(np.float32).reshape(image.height, image.width)

    # point_depth_converted_float32 = depth_conversion(point_depth_float32, point_depth_float32.shape[1] / 2)
    # point_depth_converted_uint8 = point_depth_converted_float32.view(np.uint8).astype(np.uint8)

    projectionMatrix = client.simGetCameraInfo("Depth_cam").proj_mat

    point_cloud = cv2.reprojectImageTo3D(byte_array_uint8.reshape(image.height, image.width), projectionMatrix)
    savePointCloud(point_cloud, "point_cloud.asc")

    # global depth_to_plane_msg
    # depth_to_plane_msg = image
    # depth_to_plane_msg.data = np.ndarray.tolist(byte_array_uint8)

    # global point_cloud_msg
    # point_cloud_msg.header.frame_id = depth_to_plane_msg.header.frame_id
    # point_cloud_msg.header.stamp = depth_to_plane_msg.header.stamp
    # point_cloud_msg.width = depth_to_plane_msg.height * depth_to_plane_msg.width
    # point_cloud_msg.height = 1
    # point_cloud_msg.is_bigendian = False
    # point_cloud_msg.is_dense = False
    # point_cloud_msg.data = np.ndarray.tolist(point_cloud_uint8)


# rospy.init_node('ros_depth_handler', anonymous=True)
# rospy.Subscriber('/airsim_drone/Depth_cam', Image, callback) 

# depth_to_plane_msg = Image()
client = airsim.MultirotorClient()
depth_to_plane_msg = client.simGetImages([airsim.ImageRequest("Depth_cam", airsim.ImageType.DepthPlanar, False, True)])[0]
callback(depth_to_plane_msg)
# point_cloud_msg = PointCloud2()

# pub_depth_cam_plane = rospy.Publisher('/airsim_drone/Depth_cam_plane', Image, queue_size=10)
# pub_point_cloud = rospy.Publisher('/airsim_drone/point_cloud', PointCloud2, queue_size=10)

# while not rospy.is_shutdown():
#     pub_depth_cam_plane.publish(depth_to_plane_msg)
#     # pub_point_cloud.publish(point_cloud_msg)
#     depth_to_plane_msg = Image()
#     # point_cloud_msg = PointCloud2()
#     rospy.sleep(1) 