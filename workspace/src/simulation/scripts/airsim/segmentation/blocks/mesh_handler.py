import setup_path
import airsim

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import Point, Pose, PoseArray

import re
import numpy as np
import pandas as pd

###

def unreal_to_airsim(position: Point):
    new_position = Point()

    # Unreal (cm) --> Airsim (m // NED)
    new_position.x = 10**-2 * position.x
    new_position.y = 10**-2 * position.y
    new_position.z = - (10**-2 * position.z)

    return new_position

def airsim_to_ros(position: Point):
    new_position = Point()

    # Airsim (m // NED) --> ROS (m // ASL)
    new_position.x = position.x
    new_position.y = -position.y
    new_position.z = -position.z

    return new_position

###

P_ABSOLUTE_UNREAL = Point(5700, 0, 370)
P_ABSOLUTE_UNREAL_TO_AIRSIM = unreal_to_airsim(P_ABSOLUTE_UNREAL)

def get_color_meshes(img_rgb):
    # Find unique colors except last (sky white)
    colors = np.unique(img_rgb.reshape(-1, img_rgb.shape[2]), axis=0)[:-1]
    mesh_ids = np.zeros(len(colors), dtype=np.uint8)
    
    for idx, color in enumerate(colors):
        r_df = color_checks.loc[color_checks[1] == color[0]]
        g_df = r_df.loc[color_checks[2] == color[1]]
        b_df = g_df.loc[color_checks[3] == color[2]]

        mesh_ids[idx] = b_df.iloc[0][0]
    
    return mesh_ids

def get_mesh_location(mesh_ids):
    mesh_location_array = np.ndarray(shape=(len(mesh_ids)), dtype=Pose)

    p_absolute_airsim = client.simGetVehiclePose().position

    for idx, idm in enumerate(mesh_ids):

        mesh_location = Pose()
        m_relative_airsim = Point()

        # Obtain absolute location wrt Unreal World from ground truth
        mesh_name_pattern = color_name_dict.get(str(idm))
        m_absolute_unreal = name_point_dict.get(mesh_name_pattern)

        # Obtain absolute location wrt Airsim World
        m_absolute_unreal_to_airsim = unreal_to_airsim(m_absolute_unreal)

        # Transform to relative location wrt Vehicle
        m_relative_airsim.x = m_absolute_unreal_to_airsim.x  + (p_absolute_airsim.x_val - P_ABSOLUTE_UNREAL_TO_AIRSIM.x)
        m_relative_airsim.y = m_absolute_unreal_to_airsim.y  + (p_absolute_airsim.y_val - P_ABSOLUTE_UNREAL_TO_AIRSIM.y)
        m_relative_airsim.z = m_absolute_unreal_to_airsim.z  + (p_absolute_airsim.z_val - P_ABSOLUTE_UNREAL_TO_AIRSIM.z)

        mesh_location.position = airsim_to_ros(m_relative_airsim)
        mesh_location_array[idx] = mesh_location

    return mesh_location_array

###

def callback(image):
    img1d = np.frombuffer(np.array(image.data), dtype=np.uint8) #get numpy array
    img_rgb = img1d.reshape(image.height, image.width, 3) #reshape array to 3 channel image array H X W X 3
    
    global mesh_ids_msg
    mesh_ids_msg.data = get_color_meshes(img_rgb).tolist()

    global mesh_location_msg
    mesh_location_msg.poses = get_mesh_location(mesh_ids_msg.data)

###

client = airsim.client.MultirotorClient()

mesh_names_regex = ['cl', 'cr','cy[\w]*',
                    'ob[\w]bl', 'ob[\w]br', 'ob[\w]f', 'g[\w]*',
                    'bwb[\w]tcr[\w]*', 'bwf[\w]tcr[\w]*', 
                    'fwb[\w]tcr[\w]*', 'fwf[\w]tcr[\w]*', 
                    'lwb[\w]tcr[\w]*', 'lwf[\w]tcr[\w]*', 
                    'rwb[\w]tcr[\w]*', 'rwf[\w]tcr[\w]*']
mesh_position_buffers = client.simGetMeshPositionVertexBuffers()

color_name_dict = {}
name_point_dict = {}

client.simSetSegmentationObjectID('skyshereblueprint', 255, True) # Set white sky
for idx, mesh_pattern in enumerate(mesh_names_regex):
    color_id = idx * 15
    client.simSetSegmentationObjectID(mesh_pattern, color_id, True)
    color_name_dict.update({str(color_id): mesh_pattern})

    average_point = Point()
    total_points = 0
    for buffered_mesh in mesh_position_buffers:
        if re.search(mesh_pattern, buffered_mesh.name):
            average_point.x += buffered_mesh.position.x_val
            average_point.y += buffered_mesh.position.y_val
            average_point.z += buffered_mesh.position.z_val

            total_points += 1

    average_point.x /= total_points
    average_point.y /= total_points
    average_point.z /= total_points

    name_point_dict.update({mesh_pattern: average_point})

color_checks_path = '/media/student/New Volume/fernand0labra/ros-airsim-compatibility/docs/segmentation/seg_rgbs.txt'
color_checks = pd.read_csv(color_checks_path, sep = ' ', header=None)

mesh_ids_msg = UInt8MultiArray()
mesh_location_msg = PoseArray()

# image_response: airsim.ImageResponse = client.simGetImages([airsim.ImageRequest("Seg_cam", 5, False, False)])[0]
# image = np.array(bytearray(image_response.image_data_uint8)).astype(np.uint8).reshape(image_response.height, image_response.width, 3)

rospy.init_node('mesh_handler', anonymous=True)
rospy.Subscriber('/airsim_drone/Scene_cam', Image, callback)

pub_mesh_ids = rospy.Publisher('/airsim_drone/mesh_ids', UInt8MultiArray, queue_size=10)
pub_mesh_location = rospy.Publisher('/airsim_drone/mesh_location', PoseArray, queue_size=10)

while not rospy.is_shutdown():
    pub_mesh_ids.publish(mesh_ids_msg)
    pub_mesh_location.publish(mesh_location_msg)
    
    mesh_ids_msg = UInt8MultiArray()
    mesh_location_msg = PoseArray()
    
    rospy.sleep(1)