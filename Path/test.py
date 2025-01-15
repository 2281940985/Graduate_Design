import matplotlib.pyplot as plt
import numpy as np

# Define grid size and center point
grid_size = 70
center = (65, 65)

# Create the grid plot
fig, ax = plt.subplots(figsize=(6, 6))

# Set the limits of the plot
ax.set_xlim(0, grid_size)
ax.set_ylim(0, grid_size)
ax.set_xticks(np.arange(0, grid_size, 1))
ax.set_yticks(np.arange(0, grid_size, 1))
ax.grid(True, which='both')

# Highlight the 3x3 grid surrounding the center
x_center, y_center = center

# Coordinates for the outer 9 grids (including the center)
x_min, x_max = x_center - 1, x_center + 1
y_min, y_max = y_center - 1, y_center + 1

# Draw the red dashed boundary for the outer 9 grids
ax.plot([x_min, x_min], [y_min, y_max], 'red', lw=1)  # left boundary
ax.plot([x_max, x_max], [y_min, y_max], 'red', lw=1)  # right boundary
ax.plot([x_min, x_max], [y_min, y_min], 'red', lw=1)  # bottom boundary
ax.plot([x_min, x_max], [y_max, y_max], 'red', lw=1)  # top boundary

# Display the grid
plt.gca().invert_yaxis()  # Invert the y-axis to match coordinate system
plt.show()
