# In settings.json first activate computer vision mode:
# https://github.com/Microsoft/AirSim/blob/main/docs/image_apis.md#computer-vision-mode

import setup_path
import airsim

import time
import math
import numpy as np

def generate_circle_points(radius, n):
    points = []
    for i in range(n):
        angle = 2 * math.pi * i / n  # Calculate the angle for the current point
        x = radius * math.cos(angle)  # Calculate the x-coordinate
        y = radius * math.sin(angle)  # Calculate the y-coordinate
        points.append((x, y))  # Add the point to the list
    return points

client = airsim.VehicleClient()
client.confirmConnection()

camera_pose_0 = airsim.Pose(airsim.Vector3r(-5, 0, 0), airsim.to_quaternion(0, 0, 0))
camera_pose_1 = airsim.Pose(airsim.Vector3r(5, 0, 0), airsim.to_quaternion(0, 0, 0))

client.simSetCameraPose("0", camera_pose_0)
client.simSetCameraPose("1", camera_pose_1)

###

height = 10
radius = 5

num_points_meter = 10
num_points_circle = round(num_points_meter * 2 * math.pi * radius)

height_points = np.linspace(0.0, height, round(height) * num_points_meter)

sleep_time = 0.05

for z in height_points:
    client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(0, 0, -z), airsim.to_quaternion(0, 0, 0)), True)
    time.sleep(sleep_time)

time.sleep(2)

for (z, y) in generate_circle_points(radius, num_points_circle):
    client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(0, y, -(z+5)), airsim.to_quaternion(0, 0, 0)), True)
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Segmentation),
        airsim.ImageRequest("1", airsim.ImageType.Segmentation)])

time.sleep(2)

for z in reversed(height_points):
    client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(0, 0, -z), airsim.to_quaternion(0, 0, 0)), True)
    time.sleep(sleep_time)
