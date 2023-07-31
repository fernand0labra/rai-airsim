import rospy

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray
from geometry_msgs.msg import PoseArray
from simulation.msg import Airsim

def callback(data):
    if isinstance(data, Odometry):
        airsimMsg.odom_gt = data

    elif isinstance(data, Image):
        airsimMsg.seg_img = data
    

    elif isinstance(data, UInt8MultiArray):
        airsimMsg.mesh_ids = data 

    elif isinstance(data, PoseArray):
        airsimMsg.mesh_location = data 


global airsimMsg

rospy.init_node('dataset_collector', anonymous=True)

rospy.Subscriber('/airsim_drone/ground_truth/odometry', Odometry, callback)     # odom_gt
rospy.Subscriber('/airsim_drone/Scene_cam', Image, callback)                    # seg_img
rospy.Subscriber('/airsim_drone/mesh_ids', UInt8MultiArray, callback)           # mesh_ids
rospy.Subscriber('/airsim_drone/mesh_location', PoseArray, callback)            # mesh_location

airsimMsg = Airsim()
pub = rospy.Publisher('/airsim_drone/segmentation', Airsim, queue_size=10)

while not rospy.is_shutdown():
    pub.publish(airsimMsg)
    airsimMsg = Airsim()
    rospy.sleep(1) 