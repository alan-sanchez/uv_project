#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D

# Read the CSV file into a pandas DataFrame
df = pd.read_csv('mug_3.csv')

# Extract the x, y, and z coordinates
x = df['X_pos']
y = df['Y_pos']
z = df['Z_pos']
color = df['UV_dose']

# Define the desired range for the color mapping
color_min = 0.0
color_max = 151.66

# Define a colormap with reversed order
cmap = plt.cm.get_cmap('RdYlGn')

# Create the figure and axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the points
scatter = ax.scatter(x, y, z, c=color, cmap=cmap, vmin=color_min, vmax=color_max)

# Set plot labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Add a color bar
cbar = fig.colorbar(scatter)
cbar.set_label('UV_dose (W/m^2)')

# Show the plot
plt.show()
