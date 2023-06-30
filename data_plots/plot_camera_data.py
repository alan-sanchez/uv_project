#!/usr/bin/env python3

import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Get the list of CSV files
csv_files = glob.glob('*.csv')

# Dictionary to store the y-values for each x-value
data = {}

# Read and process each CSV file
for file in csv_files:
    df = pd.read_csv(file)
    x_values = df['y']
    y_values = df['u']

    for x, y in zip(x_values, y_values):
        if x in data:
            data[x].append(y)
        else:
            data[x] = [y]

# Compute the average y-value for each x
averages = {x: sum(y_values) / len(y_values) for x, y_values in data.items()}

y_axis = np.linspace(0.5,-0.5,14)
uv_dose_sensor_array = [363.84,	233.42,	186.98,	156.38,	162.70,	195.86,	372.14,	190.35,	187.01,	172.47,	170.67,	198.43,	253.32,	315.29]


# Plot the results
plt.figure()
plt.plot(averages.keys(), averages.values(), 'bo', label='Camera Data')
plt.plot(y_axis, uv_dose_sensor_array, 'ro-', label="Sensor Array Data")
plt.xlabel('Y axis (meters)')
plt.ylabel('Average UV Dose (w/m^2)')
plt.title('Average UV dose for ten 1D Passes')
plt.grid()
plt.legend()
plt.show()