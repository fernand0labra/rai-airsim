import setup_path
import airsim

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

client = airsim.client.MultirotorClient()

client.simPause(True)
response = client.simGetImages([airsim.ImageRequest(0, airsim.ImageType.Segmentation)])[0]
client.simPause(False)

img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) #get numpy array
img_rgb = img1d.reshape(response.height, response.width, 3) #reshape array to 3 channel image array H X W X 3

#find unique colors
r = np.unique(img_rgb[:,:,0], return_index=True)
g = np.unique(img_rgb[:,:,1], return_index=True)
b = np.unique(img_rgb[:,:,2], return_index=True)  

colors = {}
for idx in r[1]:
    colors.update({str(idx): [0, 0, 0]})

# array_idx     ->  Index for the position of the color [r, g, b] = [0, 1, 2]
# color_array   ->  Array of color values and indices of values in the original image
for array_idx, color_array in enumerate([r, g, b]):
    # idx   ->  Indices of values in the original image
    for idx in range(len(color_array[1])):
        color = colors.get(str(color_array[1][idx]))  # Array of reconstructed color
        color[array_idx] = color_array[0][idx]  # Set color value
        colors.update({str(color_array[1][idx]): color})
        
print(list(colors.values()))

color_checks_path = '/media/student/New Volume/fernand0labra/ros-airsim-compatibility/workspace/src/simulation/docs/seg_rgbs.txt'
color_checks = pd.read_csv(color_checks_path, sep = '\t', header=None)

### Colors are not the same