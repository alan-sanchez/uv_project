#!/usr/bin/env python3
import glob
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


# Use glob.glob to retrieve a list of pickle file paths
csv_filenames = glob.glob('*.csv')

# Create a grid of subplots
fig, axes = plt.subplots(3, 4, figsize=(12, 9))

for idx, csv_filename in enumerate(csv_filenames):
    row = idx // 4
    col = idx % 4
    
    ## Read the pickle file into a pandas DataFrame
    df = pd.read_csv(csv_filename)

    ## Extract the UV_dose coordinates
    values = df['UV_dose'] / 156.11

    # Define custom bin edges
    bin_width = .25  # Set your desired bin width
    max_value = max(values)
    bin_edges = [i * bin_width for i in range(int(max_value / bin_width + 2))]

    ## Plot the histogram in the corresponding subplot
    axes[row, col].axvline(x=1, color='red', linestyle='--', label='Required UV dose')
    n, bins, patches = axes[row, col].hist(values, bins=bin_edges, edgecolor='black')

    bin_width = bins[1] - bins[0]
    bin_centers = bins[:-1] + bin_width / 2
    num_inactive_cells = 0
    for count, x in zip(n, bin_centers):
        axes[row, col].text(x, count, str(int(count)), ha='center', va='bottom')
        if x > 0.9:
            num_inactive_cells += count
    axes[row, col].set_title('Histogram ' + csv_filename + ' '+str(int(num_inactive_cells)))
    axes[row, col].set_xlabel('Percentage of the required UV dose')
    axes[row, col].set_ylabel('Frequency')
    axes[row, col].grid(axis='y')
    axes[row, col].legend()

# Adjust layout and display the plot
plt.tight_layout()
plt.show()