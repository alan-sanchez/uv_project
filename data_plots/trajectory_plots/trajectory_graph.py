import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from transforms3d.euler import quat2mat, quat2euler

# Load the pickle file containing recorded poses
filename = 'FB_mug_3.pkl'  # Replace with your actual filename
with open(filename, 'rb') as f:
    recorded_poses = pickle.load(f)

rounded_list = [[[round(item, 2) for item in inner_list] for inner_list in middle_list] for middle_list in recorded_poses]



# Extract x, y, and z components of translation
x = [item[0][0] for item in rounded_list]
y = [item[0][1] for item in rounded_list]
z = [item[0][2] for item in rounded_list]

# # Extract orientation quaternions and convert them to roll, pitch, yaw angles
vector_tips = [item[1] for item in rounded_list]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the translation points
ax.scatter(x, y, z, c='b', marker='o', label='End Effector Translation')

# Scale factor for the orientation arrows
arrow_scale = .5  # Adjust this value to make the arrows smaller or larger



# Plot the orientations as vectors
for i, tip in enumerate(vector_tips):
    if i == 0:
        ax.quiver(x[i], y[i], z[i],
                  arrow_scale*(tip[0]-x[i]),
                  arrow_scale*(tip[1]-y[i]), 
                  arrow_scale*(tip[2]-z[i]),  
                  label='Orientation', color='r', alpha=0.5)
    else:
        ax.quiver(x[i], y[i], z[i],
                  arrow_scale*(tip[0]-x[i]),
                  arrow_scale*(tip[1]-y[i]), 
                  arrow_scale*(tip[2]-z[i]),  
                  color='r', alpha=0.5)


# Set labels for the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Set title and legend
ax.set_title('End Effector Translation and Orientation')
ax.legend()

# Show the plot
plt.show()
