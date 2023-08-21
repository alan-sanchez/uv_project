#!/usr/bin/env python3

import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from collections import OrderedDict


class Plotter():
    def __init__(self):
        ## Get the list of CSV files
        self.camera_files = glob.glob("camera*")

        ## 
        self.y_axis = np.linspace(0.5, -0.5, 14)
        self.average_sensor_measurements = [250.18, 233.42, 186.98, 156.38, 162.70, 195.86, 372.14,	
                                            190.35, 187.01, 172.47, 170.67, 198.43, 253.32, 315.29]
        
        self.std_dev = [10.57, 9.54, 6.6, 5.01, 6.16, 6.4, 13.87, 7.26, 7.34, 6.91, 7.01, 8.78, 11.20, 13.40]
    def plot_all_data(self):
        data = {}
        ## Read and process each CSV file
        for file in self.camera_files:
            df_camera = pd.read_csv(file, sep='\t')
            x_values = df_camera.iloc[:,0]
            y_values = df_camera.iloc[:,1]
            plt.plot(x_values, y_values, label=file)

        df_sensor = pd.read_csv("UV_sensor_data.csv", sep='\t')
        for index, row in df_sensor.iterrows():
            plt.plot(self.y_axis,row, label="sensor_data_" + str(index+1))

        ## Plot the results
        plt.xlabel('Y axis (meters)')
        plt.ylabel('UV Dose ($W/m^2$)')
        plt.title('All of the Camera and Sensor Data')
        plt.grid()
        plt.legend()
        plt.show()

        
    def plot_camera_data(self):
        data = {}
        ## Read and process each CSV file
        for file in self.camera_files:
            df_camera = pd.read_csv(file, sep='\t')
            x_values = df_camera.iloc[:,0]
            y_values = df_camera.iloc[:,1]
            plt.plot(x_values, y_values, label=file)
        
        ## Plot the results
        plt.xlabel('Y axis (meters)')
        plt.ylabel('UV Dose ($W/m^2$)')
        plt.title('Camera Data of Ten 1D passes')
        plt.grid()
        plt.legend()
        plt.show()


    def plot_sensor_data(self):
        df_sensor = pd.read_csv("UV_sensor_data.csv", sep='\t')
        for index, row in df_sensor.iterrows():
            plt.plot(self.y_axis,row, label="sensor_data_" + str(index+1))

        ## Plot the results
        plt.xlabel('Y axis (meters)')
        plt.ylabel('UV Dose ($W/m^2$)',fontsize=14)
        plt.title('Sensor Data of Ten 1D passes')
        plt.grid()
        plt.legend()
        plt.show()


    def plot_camera_with_sensor_average(self):
        data = {}
        ## Read and process each CSV file
        iter = 1
        for file in self.camera_files:
            df_camera = pd.read_csv(file, sep='\t')
            x_values = df_camera.iloc[:,0]
            y_values = df_camera.iloc[:,1]
            plt.plot(x_values, y_values, label="RealSense Data Trial "+str(iter))
            iter += 1

        ## Plot the results
        plt.errorbar(self.y_axis, self.average_sensor_measurements,yerr=self.std_dev, capsize=5, fmt='o-', color='red', ecolor='black', label="Average UV Sensor Array Data")
        plt.xlabel('Y axis (meters)', fontsize=16)
        plt.ylabel('Average UV Dose ($W/m^2$)', fontsize=16)
        plt.title('Camera Data with Average UV Sensor Data', fontsize = 24)
        plt.grid()
        plt.legend()
        plt.show()


    def plot_averages(self):
        data = {}
        ## Read and process each CSV file
        for file in self.camera_files:
            ##    
            df_camera = pd.read_csv(file, sep='\t')
            x_values = df_camera.iloc[:,0]
            y_values = df_camera.iloc[:,1]

            ##
            for x, y in zip(x_values, y_values):
                if x in data:
                    data[x].append(y)
                else:
                    data[x] = [y]

        ## Compute the average y-value for each x
        averages = {x: sum(y_values) / len(y_values) for x, y_values in data.items()}
        std_devs = {x: np.std(y_values) for x, y_values in data.items()}  # Calculate standard deviation

        ordered_data = OrderedDict(sorted(averages.items()))
        # Extract x, y average, and standard deviation values for plotting
        x_vals = list(ordered_data.keys())
        y_avgs = list(ordered_data.values())
        y_std = [std_devs[x] for x in ordered_data.keys()]  # Retrieve standard deviations in the same order

        ## Plot the results
        plt.figure()
        plt.errorbar(ordered_data.keys(), ordered_data.values(), yerr=y_std, capsize=5, fmt='-', color='blue',ecolor='black', label='Average RealSense Data')
        plt.errorbar(self.y_axis, self.average_sensor_measurements,yerr=self.std_dev, capsize=5, fmt='o-', color='red', ecolor='black', label="Average UV Sensor Array Data")
        plt.xlabel('Y axis (meters)', fontsize=16)
        plt.ylabel('UV Dose ($W/m^2$)', fontsize=16)
        plt.title('Average Camera and UV Sensor Data for Ten 1D Passes', fontsize =24)
        plt.grid()
        plt.legend()
        plt.show()


if __name__ == '__main__':
    model = Plotter()
    model.plot_averages()
    # model.plot_all_data()
    # model.plot_camera_data()
    # model.plot_sensor_data()
    model.plot_camera_with_sensor_average()