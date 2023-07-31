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
        

    def plot_all_data(self):
        data = {}
        # Read and process each CSV file
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
        plt.ylabel('UV Dose (w/m^2)')
        plt.title('All of the Camera and Sensor Data')
        plt.grid()
        plt.legend()
        plt.show()

        
    def plot_camera_data(self):
        data = {}
        # Read and process each CSV file
        for file in self.camera_files:
            df_camera = pd.read_csv(file, sep='\t')
            x_values = df_camera.iloc[:,0]
            y_values = df_camera.iloc[:,1]
            plt.plot(x_values, y_values, label=file)
        
        # # Plot the results
        plt.xlabel('Y axis (meters)')
        plt.ylabel('UV Dose (w/m^2)')
        plt.title('Camera Data of Ten 1D passes')
        plt.grid()
        plt.legend()
        plt.show()


    def plot_sensor_data(self):
        df_sensor = pd.read_csv("UV_sensor_data.csv", sep='\t')
        for index, row in df_sensor.iterrows():
            plt.plot(self.y_axis,row, label="sensor_data_" + str(index+1))

        # # Plot the results
        plt.xlabel('Y axis (meters)')
        plt.ylabel('UV Dose (w/m^2)')
        plt.title('Sensor Data of Ten 1D passes')
        plt.grid()
        plt.legend()
        plt.show()


    def plot_camera_with_sensor_average(self):
        data = {}
        ## Read and process each CSV file
        for file in self.camera_files:
            df_camera = pd.read_csv(file, sep='\t')
            x_values = df_camera.iloc[:,0]
            y_values = df_camera.iloc[:,1]
            plt.plot(x_values, y_values, label=file)

        ## Plot the results
        plt.plot(self.y_axis, self.average_sensor_measurements, 'ro-', label="Sensor Array Data")
        plt.xlabel('Y axis (meters)')
        plt.ylabel('Average UV Dose (w/m^2)')
        plt.title('Camera Data of Ten 1D Passes and Average Sensor Data')
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

        ordered_data = OrderedDict(sorted(averages.items()))

        ## Plot the results
        plt.figure()
        plt.plot(ordered_data.keys(), ordered_data.values(), 'b', label='Camera Data')
        plt.plot(self.y_axis, self.average_sensor_measurements, 'ro-', label="Sensor Array Data")
        plt.xlabel('Y axis (meters)')
        plt.ylabel('UV Dose (w/m^2)')
        plt.title('Average UV Dose For Ten 1D Passes')
        plt.grid()
        plt.legend()
        plt.show()


if __name__ == '__main__':
    model = Plotter()
    model.plot_averages()
    model.plot_all_data()
    model.plot_camera_data()
    model.plot_sensor_data()
    model.plot_camera_with_sensor_average()