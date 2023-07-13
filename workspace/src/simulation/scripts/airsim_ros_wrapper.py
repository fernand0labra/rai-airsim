import rospy

from sensor_msgs.msg import Image, Imu, PointCloud2
from nav_msgs.msg import Odometry
from simulation.msg import Airsim

def callback(data):
    if isinstance(data, Image):
        airsimMsg.seg_img = data
    
    elif isinstance(data, Imu):
            airsimMsg.imu = data

    elif isinstance(data, PointCloud2):
            airsimMsg.lidar = data

    elif isinstance(data, Odometry):
            airsimMsg.odom_gt = data

global airsimMsg

rospy.init_node('ros_wrapper_client', anonymous=True)

rospy.Subscriber('/airsim_node/Drone_1/odom_local_ned', Odometry, callback)
rospy.Subscriber('/airsim_node/Drone_1/camera_1/Segmentation', Image, callback)
rospy.Subscriber('/airsim_node/Drone_1/imu/', Imu, callback)
rospy.Subscriber('/airsim_node/Drone_1/lidar/', PointCloud2, callback)

pub = rospy.Publisher('/ros_wrapper_client/airsim', Airsim, queue_size=10)
airsimMsg = Airsim()

while not rospy.is_shutdown():
    pub.publish(airsimMsg)
    airsimMsg = Airsim()
    rospy.sleep(1) 