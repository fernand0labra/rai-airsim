import numpy as np
import rospy
import math
from sensor_msgs.msg import Image


def callback(image):
    global depth_to_plane_msg
    numpy_array = np.frombuffer(np.array(image.data), dtype=float)
    point_depth = numpy_array.reshape(image.height, image.width)

    depth_to_plane_msg = depth_conversion(point_depth, point_depth.shape[1] / 2)

def depth_conversion(point_depth, f):
    H = point_depth.shape[0]
    W = point_depth.shape[1]
    i_c = np.float(H) / 2 - 1
    j_c = np.float(W) / 2 - 1
    columns, rows = np.meshgrid(np.linspace(0, W-1, num=W), np.linspace(0, H-1, num=H))
    distance_from_center = ((rows - i_c)**2 + (columns - j_c)**2)**(0.5)
    plane_depth = point_depth / (1 + (distance_from_center / f)**2)**(0.5)
    return plane_depth


rospy.init_node('ros_depth_handler', anonymous=True)
rospy.Subscriber('/airsim_drone/Depth_cam', Image, callback) 

depth_to_plane_msg = Image()
pub = rospy.Publisher('/airsim_drone/Depth_cam_to_scene', Image, queue_size=10)

while not rospy.is_shutdown():
    pub.publish(depth_to_plane_msg)
    depth_to_plane_msg = Image()
    rospy.sleep(1) 