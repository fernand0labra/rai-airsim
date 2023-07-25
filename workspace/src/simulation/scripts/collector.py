import rospy

from sensor_msgs.msg import Image, PointCloud2
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

rospy.init_node('airsim_publisher', anonymous=True)

rospy.Subscriber('/airsim_drone/ground_truth/odometry', Odometry, callback)     # odom_gt
rospy.Subscriber('/airsim_drone/Seg_cam', Image, callback)                      # seg_img
rospy.Subscriber('/airsim_drone/mesh_ids', UInt8MultiArray, callback)           # mesh_ids
rospy.Subscriber('/airsim_drone/RGBD_cam', PointCloud2, callback)               # mesh_location

airsimMsg = Airsim()
pub = rospy.Publisher('/unreal_airsim_collector', Airsim, queue_size=10)

while not rospy.is_shutdown():
    pub.publish(airsimMsg)
    airsimMsg = Airsim()
    rospy.sleep(1) 