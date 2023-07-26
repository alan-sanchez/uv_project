#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def plot_cube(ax, x, y, z, color):
    ## Define cube dimensions
    dx = dy = dz = 0.01
    
    ## Define cube position
    x -= dx / 2
    y -= dy / 2
    z -= dz / 2
    
    ## Plot the cube
    ax.bar3d(x, y, z, dx, dy, dz, color=color)

## Read the CSV file into a pandas DataFrame
df = pd.read_csv('test5.csv')

## Extract the x, y, and z coordinates
x = df['X_pos']
y = df['Y_pos']
z = df['Z_pos']
color = df['UV_dose']

## Define the desired range for the color mapping
color_min = 0.0
color_max = 151.66

## Define a colormap with reversed order
cmap = plt.cm.get_cmap('RdYlGn')

## Create the figure and axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

## Plot the cubes with colors and custom color range
for xi, yi, zi, color_val in zip(x, y, z, color):
    cube_color = cmap((color_val - color_min) / (color_max - color_min))
    plot_cube(ax, xi, yi, zi, cube_color)

## Plot the points
# scatter = ax.scatter(x, y, z, c=color, cmap=cmap, vmin=color_min, vmax=color_max)

## Set plot labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

## Set equal spacing for the grid lines
ax.set_box_aspect([.3,.5,.9])

## Add a color bar
# cbar = fig.colorbar(scatter)
cbar = fig.colorbar(plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=color_min, vmax=color_max)))
cbar.set_label('UV_dose (W/m^2)')

## Show the plot
plt.show()
