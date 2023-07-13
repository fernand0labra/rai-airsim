import setup_path
import airsim

import math
import time

def apply_scale(z, y):
    return (y+SHIFT)*FACTOR, -(z+SHIFT)*FACTOR

### Constants
SLEEP = 1
SPEED = 2
FACTOR = 2
SHIFT = 2

SQRT_3 = math.sqrt(3)

# Hexagon vertice coordinates with side of 1
Z = [SQRT_3/2, 0, -SQRT_3/2, -SQRT_3/2, 0, SQRT_3/2]
Y = [-1/2, -1, -1/2, 1/2, 1, 1/2]

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

imu_data = client.getImuData()
lidar_data = client.getLidarData()

client.armDisarm(True)
client.takeoffAsync().join()
time.sleep(5)

# Move to the center
client.moveToPositionAsync(0, *apply_scale(0, 0), SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

for (z, y) in zip(Z, Y):
    client.moveToPositionAsync(0, *apply_scale(round(z, 2), round(y, 2)), SPEED).join()
    client.hoverAsync().join()
    time.sleep(SLEEP)

client.moveToPositionAsync(0, *apply_scale(0, 0), SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Segmentation),
    airsim.ImageRequest("1", airsim.ImageType.Segmentation)])  #segmentation

client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
