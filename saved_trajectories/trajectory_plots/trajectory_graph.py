import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from transforms3d.euler import quat2mat, quat2euler

# Load the pickle file containing recorded poses
filename = 'recorded_poses.pkl'  # Replace with your actual filename
with open(filename, 'rb') as f:
    recorded_poses = pickle.load(f)

rounded_list = [[[round(item, 2) for item in inner_list] for inner_list in middle_list] for middle_list in recorded_poses]

print(rounded_list[0])
# filtered_list = [item for item in recorded_poses if item is not None]

# Extract x, y, and z components of translation
x = [item[0][0] for item in rounded_list]
y = [item[0][1] for item in rounded_list]
z = [item[0][2] for item in rounded_list]

# # Extract orientation quaternions and convert them to roll, pitch, yaw angles
vector_tips = [item[1] for item in rounded_list]
# roll, pitch, yaw = zip(*[quat2euler(orientation) for orientation in quats])
# print(vector_tips)
# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the translation points
ax.scatter(x, y, z, c='b', marker='o', label='End Effector Translation')

# # Scale factor for the orientation arrows
# arrow_scale = 0.05  # Adjust this value to make the arrows smaller or larger

# # Plot the orientation angles with smaller arrows
# ax.quiver(x, y, z, arrow_scale * np.cos(yaw), arrow_scale * np.sin(yaw), arrow_scale * np.zeros_like(z),
#           color='r', label='Orientation', alpha=0.5)

# Plot the orientations as vectors
for i, tip in enumerate(vector_tips):
    ax.quiver(x[i], y[i], z[i], tip[0]-x[i], tip[1]-y[i], tip[2]-z[i], color='r')
# print(x[0],y[0],z[0],vector_tips[0])
# ax.quiver(x[0], y[0], z[0], vector_tips[0], vector_tips[1], vector_tips[2], color='r')

# Set labels for the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Set title and legend
ax.set_title('End Effector Translation and Orientation')
ax.legend()

# Show the plot
plt.show()



# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Define the vector components
# x = 2
# y = 3
# z = 4

# # Create a new figure
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # Plot the vector as an arrow
# ax.quiver(0, 0, 0, x, y, z, color='b')

# # Set limits for the x, y, and z axes
# ax.set_xlim([0, max(x, y, z) + 1])
# ax.set_ylim([0, max(x, y, z) + 1])
# ax.set_zlim([0, max(x, y, z) + 1])

# # Set labels for the axes
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# # Set the title
# ax.set_title('Plot of a 3D Vector')

# # Show the plot
# plt.show()
