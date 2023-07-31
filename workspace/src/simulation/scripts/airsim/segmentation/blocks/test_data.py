import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from unreal_airsim.msg import Airsim

import numpy as np
import pandas as pd


def callback(airsim_msg):
    n_points = len(airsim_msg.mesh_ids.data)

    point_list = np.empty((3, n_points), dtype=np.float32)  # [x, y, z]
    color_list = np.empty((n_points, 4), dtype=np.float32)    # [r, g, b, alpha]

    # plt.imshow(np.array(bytearray(airsim_msg.seg_img.data), dtype=np.uint8).reshape(airsim_msg.seg_img.height, airsim_msg.seg_img.width, 3))
    # plt.show()

    fig = plt.figure()
    ax = Axes3D(fig)

    for idx in range(n_points):
        point = airsim_msg.mesh_location.poses[idx].position

        point_list[0][idx] = point.x
        point_list[1][idx] = point.y
        point_list[2][idx] = point.z

        global color_checks
        color = color_checks.iloc[airsim_msg.mesh_ids.data[idx]]
        print(color[1:3])
        print(color[1:3]/255)
        color_list[idx] = [color.iloc[1]/255, color.iloc[1]/255, color.iloc[3]/255, 1]
    
    vehicle_position = airsim_msg.odom_gt.pose.pose.position

    ax.scatter(point_list[0], point_list[1], point_list[2], c=color_list, marker='o')
    ax.scatter(vehicle_position.x, vehicle_position.y, vehicle_position.z, marker='^')
    plt.show()

color_checks_path = '/media/student/New Volume/fernand0labra/ros-airsim-compatibility/docs/segmentation/seg_rgbs.txt'
color_checks = pd.read_csv(color_checks_path, sep = ' ', header=None)

rospy.init_node('test_data', anonymous=True)
rospy.Subscriber('/airsim_drone/segmentation', Airsim, callback, queue_size=1)

rospy.spin()