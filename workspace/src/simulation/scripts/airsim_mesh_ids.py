import setup_path
import airsim

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


global mesh_ids

def callback(image):
    img1d = np.frombuffer(np.array(image.data), dtype=np.uint8) #get numpy array
    img_rgb = img1d.reshape(image.height, image.width, 3) #reshape array to 3 channel image array H X W X 3

    #find unique colors
    r = np.unique(img_rgb[:,:,0], return_index=True)
    g = np.unique(img_rgb[:,:,1], return_index=True)
    b = np.unique(img_rgb[:,:,2], return_index=True)  

    colors = {}
    for idx in r[1]:
        colors.update({str(idx): [0, 0, 0]})

    # array_idx     ->  Index for the position of the color [r, g, b] = [0, 1, 2]
    # color_array   ->  Array of color values and indices of values in the original image
    for array_idx, color_array in enumerate([r, g, b]):
        # idx   ->  Indices of values in the original image
        for idx in range(len(color_array[1])):
            color = colors.get(str(color_array[1][idx]))  # Array of reconstructed color
            color[array_idx] = color_array[0][idx]  # Set color value
            colors.update({str(color_array[1][idx]): color})

    print(colors)

    global mesh_ids
    mesh_ids = np.zeros(len(colors.keys()), dtype=np.uint8)
    
    for idx, color in enumerate(colors.values()):
        r_df = color_checks.loc[color_checks[1] == color[0]]
        g_df = r_df.loc[color_checks[2] == color[1]]
        b_df = g_df.loc[color_checks[3] == color[2]]

        mesh_ids[idx] = b_df.iloc[0][0]
    
    for mesh_name in mesh_names:
        client.simGetSegmentationObjectID(mesh_name)


client = airsim.client.MultirotorClient()

mesh_names_regex = ['cone[\w]*', 'templatecube[\w]rounded[\w]*', 'orangeball', 'ground[\w]*', 'skyshereblueprint']
mesh_names = ['cone_5', 'templatecube_rounded_10', 'orangeball', 'ground_2', 'skyshereblueprint']

for idx, mesh_name in enumerate(mesh_names_regex):
    client.simSetSegmentationObjectID(mesh_name, idx*50, True)

color_checks_path = '/media/student/New Volume/fernand0labra/ros-airsim-compatibility/workspace/src/simulation/docs/seg_rgbs.txt'
color_checks = pd.read_csv(color_checks_path, sep = ' ', header=None)

mesh_ids = np.zeros(0, dtype=np.uint8)
mesh_ids_msg = UInt8MultiArray()

rospy.init_node('ros_mesh_handler', anonymous=True)
rospy.Subscriber('/airsim_drone/Seg_cam', Image, callback)
pub = rospy.Publisher('/airsim_drone/mesh_ids', UInt8MultiArray, queue_size=10)

while not rospy.is_shutdown():
    mesh_ids_msg.data = list(mesh_ids.__array__())
    pub.publish(mesh_ids_msg)
    mesh_ids = np.zeros(0, dtype=np.uint8)
    mesh_ids_msg = UInt8MultiArray()
    rospy.sleep(1) 