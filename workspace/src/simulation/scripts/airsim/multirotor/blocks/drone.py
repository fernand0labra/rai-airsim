import setup_path
import airsim

import time

### Constants
SLEEP = 2
SPEED = 3


# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

client.armDisarm(True)
client.takeoffAsync().join()
time.sleep(SLEEP)

# Ascend
position = (0, 0, -5)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Rotate 90 degrees (Right)
client.rotateToYawAsync(90).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Move to the Right
position = (0, 30, -5)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Rotate 90 degrees (Right)
client.rotateToYawAsync(180).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Move Backwards
position = (-20, 30, -5)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Rotate 90 degrees (Right)
client.rotateToYawAsync(270).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Move to the Left
position = (-20, 0, -5)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Rotate 90 degrees (Right)
client.rotateToYawAsync(0).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Move Forward
position = (0, 0, -5)# Rotate 180 degrees (Back)
# client.rotateToYawAsync(90).join()
# client.hoverAsync().join()
# time.sleep(SLEEP)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

# Descend
position = (0, 0, 0)
client.moveToPositionAsync(*position, SPEED).join()
client.hoverAsync().join()
time.sleep(SLEEP)

client.armDisarm(False)
client.enableApiControl(False)
