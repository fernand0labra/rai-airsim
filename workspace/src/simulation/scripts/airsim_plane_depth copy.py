import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image, PointCloud2
import matplotlib.pyplot as plt

color = (0,255,0)
rgb = "%d %d %d" % color
projectionMatrix = np.array([[-0.501202762, 0.000000000, 0.000000000, 0.000000000],
                              [0.000000000, -0.501202762, 0.000000000, 0.000000000],
                              [0.000000000, 0.000000000, 10.00000000, 100.00000000],
                              [0.000000000, 0.000000000, -10.0000000, 0.000000000]], dtype=np.float32)

def depth_conversion(point_depth, f):
    H = point_depth.shape[0]
    W = point_depth.shape[1]
    i_c = np.float32(H) / 2 - 1
    j_c = np.float32(W) / 2 - 1
    columns, rows = np.meshgrid(np.linspace(0, W-1, num=W, dtype=np.float32), np.linspace(0, H-1, num=H, dtype=np.float32))
    distance_from_center = ((rows - i_c)**2 + (columns - j_c)**2)**(0.5)
    plane_depth = point_depth / (1 + (distance_from_center / f)**2)**(0.5)
    return plane_depth

def callback(image):

    numpy_array_float32 = np.frombuffer(np.array(image.data), dtype=np.float32)
    point_depth = numpy_array_float32.reshape(image.height, image.width)

    depth_converted_float32 = depth_conversion(point_depth, point_depth.shape[1] / 2)
    depth_converted_uint8 = depth_converted_float32.reshape(image.height*image.width).view(np.uint8).astype(np.uint8)

    # point_cloud_float32 = cv2.reprojectImageTo3D(depth_converted_uint8, projectionMatrix)
    # point_cloud_uint8 = point_cloud_float32.reshape(image.height*image.width*4, 1, 3).view(np.uint8).astype(np.uint8)

    global depth_to_plane_msg
    depth_to_plane_msg = image
    depth_to_plane_msg.data = np.ndarray.tolist(depth_converted_uint8)

    # global point_cloud_msg
    # point_cloud_msg.header.frame_id = depth_to_plane_msg.header.frame_id
    # point_cloud_msg.header.stamp = depth_to_plane_msg.header.stamp
    # point_cloud_msg.width = depth_to_plane_msg.height * depth_to_plane_msg.width
    # point_cloud_msg.height = 1
    # point_cloud_msg.is_bigendian = False
    # point_cloud_msg.is_dense = False
    # point_cloud_msg.data = np.ndarray.tolist(point_cloud_uint8)


rospy.init_node('ros_depth_handler', anonymous=True)
rospy.Subscriber('/airsim_drone/Depth_cam', Image, callback) 

depth_to_plane_msg = Image()
# point_cloud_msg = PointCloud2()

pub_depth_cam_plane = rospy.Publisher('/airsim_drone/Depth_cam_plane', Image, queue_size=10)
# pub_point_cloud = rospy.Publisher('/airsim_drone/point_cloud', PointCloud2, queue_size=10)

while not rospy.is_shutdown():
    pub_depth_cam_plane.publish(depth_to_plane_msg)
    # pub_point_cloud.publish(point_cloud_msg)
    depth_to_plane_msg = Image()
    # point_cloud_msg = PointCloud2()
    rospy.sleep(1) 