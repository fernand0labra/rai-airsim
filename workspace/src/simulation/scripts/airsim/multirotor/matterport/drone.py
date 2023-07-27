import setup_path
import airsim

import time


### Constants
SLEEP = 2
SPEED = 10


# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

client.armDisarm(True)
client.takeoffAsync().join()
time.sleep(SLEEP)

# Ascend
position = (0, 0, -10)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Move Forward
position = (15, 0, -10)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Rotate 180 degrees
client.rotateToYawAsync(180).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Rotate 180 degrees
client.rotateToYawAsync(180).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Move Backwards
position = (-10, 0, -10)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Rotate 90 degrees (Right)
client.rotateToYawAsync(90).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Move to the Right
position = (-10, 20, -10)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Move to the Left
position = (-10, 0, -10)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Rotate -90 degrees (Left)
client.rotateToYawAsync(-90).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Move to start position
position = (0, 0, -10)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Descend
position = (0, 0, 0)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Rotate 180 degrees (Back)
# client.rotateToYawAsync(90).join()
# client.hoverAsync().join()
# time.sleep(SLEEP)

client.armDisarm(False)
client.enableApiControl(False)
