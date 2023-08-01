import rospy
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D
from simulation.msg import Airsim

import os
import cv2
import signal
import numpy as np
import pandas as pd


FOLDER = '/home/fernand0labra/ros-airsim/docs/segmentation'
N_MSGS = 0

def generate_plot_video_opencv(signum, frame):
    global N_MSGS, FOLDER
    
    os.chdir(FOLDER)
    video = cv2.VideoWriter('airsim_plot.mp4', 0x7634706d, 3.0, (1200, 600))

    for idx in range(N_MSGS+1):
        img_path = 'point_plot' + str(idx) + '.png'
        video.write(cv2.imread(img_path))
        os.remove(img_path)
        
    video.release()

def callback(airsim_msg):
    n_points = len(airsim_msg.mesh_ids.data)

    point_list = np.empty((3, n_points), dtype=np.float32)  # [x, y, z]
    color_list = np.empty((n_points, 4), dtype=np.float32)  # [r, g, b, alpha]

    global N_MSGS
    img = np.array(bytearray(airsim_msg.seg_img.data), dtype=np.uint8).reshape(airsim_msg.seg_img.height, airsim_msg.seg_img.width, 3)
    img = img[:][:]/255

    fig = plt.figure(figsize=(12, 6))
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122, projection='3d')

    ax1.imshow(img)
    ax1.axis('off')
    ax1.set_title('Segmentation Image')


    ax2.set_xlim(-60, 60)
    ax2.set_ylim(-60, 60)
    ax2.set_zlim(-5, 8)

    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')

    ax2.set_title('3D Scatter Plot')

    for idx in range(n_points):
        point = airsim_msg.mesh_location.poses[idx].position

        point_list[0][idx] = point.x
        point_list[1][idx] = point.y
        point_list[2][idx] = point.z

        global color_checks
        color = color_checks.iloc[airsim_msg.mesh_ids.data[idx]]
        color_list[idx] = [color.iloc[1]/255, color.iloc[2]/255, color.iloc[3]/255, 1]
    
    vehicle_position = airsim_msg.odom_gt.pose.pose.position

    ax2.scatter(point_list[0], point_list[1], point_list[2], c=color_list, marker='o')
    ax2.scatter(vehicle_position.x, vehicle_position.y, vehicle_position.z, marker='^')

    plt.tight_layout()

    os.chdir(FOLDER)
    plt.savefig('point_plot' + str(N_MSGS) + '.png')
    N_MSGS += 1
    exit()

signal.signal(signal.SIGTSTP, generate_plot_video_opencv)

color_checks_path = '/home/fernand0labra/ros-airsim/docs/segmentation/seg_rgbs.txt'
color_checks = pd.read_csv(color_checks_path, sep = ' ', header=None)

rospy.init_node('test_data', anonymous=True)
rospy.Subscriber('/airsim_drone/segmentation', Airsim, callback, queue_size=10)

rospy.spin()