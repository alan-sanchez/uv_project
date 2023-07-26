#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def plot_hist(values, bins=10):
    # Create the histogram with custom bins
    # plt.hist(values,bins=bins, edgecolor='black')

    # Add a vertical line at a specific value (e.g., x=0)
    plt.axvline(x=1, color='red', linestyle='--', label='Vertical Line at x=0')

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
    
    labels = [f"{b}" if b != 2.25 else "2.5+" for b in bins]
    plt.xlabel('Values')
    plt.ylabel('Frequency')
    plt.title('Histogram of User Input Path (# of Inactivated/Green Cells: '+str(int(num_inactive_cells))+')')
    plt.xticks(bins, labels)
    plt.grid(axis='y')
    plt.show()


## Read the CSV file into a pandas DataFrame
df = pd.read_csv('output.csv')

## Extract the UV_dose coordinates
values = df['UV_dose']/156.11

# Set any value over 2.0 to 2.0 using NumPy's where function
values = np.where(values > 2.0, 2.125, values)


custom_bins = [0, .25, .50, .75, 1, 1.25, 1.5, 1.75, 2.0, +2.25]

plot_hist(values, custom_bins)

