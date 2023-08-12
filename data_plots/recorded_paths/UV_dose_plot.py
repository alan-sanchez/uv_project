#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def plot_hist(values, bins=10):
    # Add a vertical line at a specific value (e.g., x=0)
    plt.axvline(x=1, color='red', linestyle='--', label='Required UV dose')

    # Create the histogram with centered bins
    n, bins, patches = plt.hist(values, bins=bins, edgecolor='black')

    # Calculate the width of each bin
    bin_width = bins[1] - bins[0]

    # Calculate the bin centers
    bin_centers = bins[:-1] + bin_width / 2

    num_inactive_cells = 0

    # Add labels to the centers of the bins
    for count, x in zip(n, bin_centers):
        plt.text(x, count, str(int(count)), ha='center', va='bottom')
        if x >.9:
            num_inactive_cells += count
    
    plt.xlabel('Percentage of the required UV dose')
    plt.ylabel('Frequency')
    plt.title('Histogram of Hard Coded Path (# of Inactivated/Green Cells: '+str(int(num_inactive_cells))+')')
    plt.grid(axis='y')
    plt.legend()
    plt.show()


## Read the CSV file into a pandas DataFrame
df = pd.read_csv('hard_coded_cone.csv')

## Extract the UV_dose coordinates
values = df['UV_dose']/156.11

# Define custom bin edges
bin_width = 1.0  # Set your desired bin width
max_value = max(values)
bin_edges = [ i * bin_width for i in range(int(max_value/bin_width))]

##
plot_hist(values, bin_edges)

