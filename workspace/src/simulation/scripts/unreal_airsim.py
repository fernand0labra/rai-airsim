import rospy

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8MultiArray
from simulation.msg import Airsim

def callback(data):
    if isinstance(data, Image):
        airsimMsg.seg_img = data
    
    elif isinstance(data, Odometry):
        airsimMsg.odom_gt = data

    elif isinstance(data, Airsim):
        airsimMsg.mesh_ids = data 

global airsimMsg

rospy.init_node('ros_wrapper_client', anonymous=True)

rospy.Subscriber('/airsim_drone/ground_truth/odometry', Odometry, callback)     # odom_gt
rospy.Subscriber('/airsim_drone/Seg_cam', Image, callback)                      # seg_img
rospy.Subscriber('/airsim_drone/mesh_ids', UInt8MultiArray, callback)           # mesh_ids
##                                                                              # mesh_location

pub = rospy.Publisher('/ros_wrapper_client', Airsim, queue_size=10)
airsimMsg = Airsim()

while not rospy.is_shutdown():
    pub.publish(airsimMsg)
    airsimMsg = Airsim()
    rospy.sleep(1) 