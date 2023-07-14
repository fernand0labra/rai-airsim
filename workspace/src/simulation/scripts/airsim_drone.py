import setup_path
import airsim

import math
import time

def calculate_hexagon_vertices(center_x, center_y, radius, scale_factor):
    vertices = []
    angle = 2 * math.pi / 6  # Angle between two adjacent vertices of a regular hexagon

    for i in range(6):
        x = center_x + radius * scale_factor * math.cos(i * angle)
        y = center_y + radius * scale_factor * math.sin(i * angle)
        vertices.append((x, y))

    return vertices

### Constants
SLEEP = 1
SPEED = 2

SQRT_3 = math.sqrt(3)

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
client.moveToPositionAsync(0, 0, -4, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

for (z, y) in calculate_hexagon_vertices(0, -4, 2, 2):
    client.moveToPositionAsync(0, y, z, SPEED).join()
    client.hoverAsync().join()
    time.sleep(SLEEP)

client.moveToPositionAsync(0, 0, -4, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Move to the center
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Segmentation),
    airsim.ImageRequest("1", airsim.ImageType.Segmentation)])  #segmentation

client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
