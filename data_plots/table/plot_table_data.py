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
        self.sensor_y_axis = np.linspace(0.5, -0.5, 14)
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
            plt.plot(self.sensor_y_axis,row, label="sensor_data_" + str(index+1))

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
            plt.plot(self.sensor_y_axis,row, label="sensor_data_" + str(index+1))

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
            plt.plot(x_values, y_values, label="RealSense Data "+str(iter))
            iter += 1

        ## Plot the results
        plt.errorbar(self.sensor_y_axis, self.average_sensor_measurements,yerr=self.std_dev, capsize=5, fmt='o-', color='red', ecolor='black', label="Average UV Sensor Array Data")
        plt.xlabel('Y axis (meters)', fontsize=16)
        plt.ylabel('Average UV Dose ($W/m^2$)', fontsize=16)
        plt.title('Camera Data with Average UV Sensor Data', fontsize = 24)
        plt.grid()
        plt.legend(fontsize='large')
        plt.show()


    def plot_averages(self,difference=False):
        data = {}
        ## Read and process each CSV file
        for file in self.camera_files:
            ##    
            df_camera = pd.read_csv(file, sep='\t')
            camera_y_pose = df_camera.iloc[:,0]
            camera_dosage = df_camera.iloc[:,1]

            ##
            for y, d in zip(camera_y_pose, camera_dosage):
                if y in data:
                    data[y].append(d)
                else:
                    data[y] = [d]

        ## Compute the average y-value for each x)
        camera_dose_averages = {x: sum(dosage) / len(dosage) for x, dosage in data.items()}
        camera_dose_std_devs = {x: np.std(dosage) for x, dosage in data.items()}  # Calculate standard deviation

        ordered_data = OrderedDict(sorted(camera_dose_averages.items()))

        y_std = [camera_dose_std_devs[x] for x in ordered_data.keys()]  # Retrieve standard deviations in the same order

        ## Plot the results
        plt.figure()
        plt.errorbar(ordered_data.keys(), ordered_data.values(), yerr=y_std, capsize=5, fmt='-', color='blue',ecolor='black', label='Average RealSense Data')
        plt.errorbar(self.sensor_y_axis, self.average_sensor_measurements,yerr=self.std_dev, capsize=5, fmt='o-', color='red', ecolor='black', label="Average UV Sensor Array Data")
        plt.xlabel('Position (meters)', fontsize=16)
        plt.ylabel('UV Dose ($W/m^2$)', fontsize=16)
        plt.title('Average Data for Ten 1D Passes', fontsize =18) #Camera and UV Sensor
        plt.grid()
        plt.legend(fontsize='large')
        plt.show()

        if difference:
            # 
            ordered_camera_y_axis = list(ordered_data.keys())
            ordered_camera_dose_avgs = list(ordered_data.values())
    
            # Interpolate camera data to match UV sensor data lengths
            dose_camera_interp = np.interp(self.sensor_y_axis, ordered_camera_y_axis, ordered_camera_dose_avgs)
            
            # Calculate difference  
            dose_diff_low = abs(dose_camera_interp - self.average_sensor_measurements)
            # print(dose_diff)

            # Calculate percentage difference 
            dose_percent_diff_low = dose_diff_low / self.average_sensor_measurements * 100

            # Optional: Calculate and print statistics on difference 
            mean_diff = np.mean(dose_diff_low)
            min_diff = np.min(dose_diff_low)
            max_diff = np.max(dose_diff_low)

            # print("Mean Difference: ", mean_diff)
            # print("Min Difference: ", min_diff)
            # print("Max Difference: ", max_diff) 
            
            ###################### interpolation switch ##########################
            # temp = list(self.sensor_y_axis)
            # r_sensor_y_axis = temp[::-1]
            # r_avg_sensor_meas = self.average_sensor_measurements[::-1]
            # dose_sensor_interp = np.interp(ordered_camera_y_axis[::-1], r_sensor_y_axis, r_avg_sensor_meas)  

            # # # Calculate difference  
            # dose_diff_high = abs(dose_sensor_interp - ordered_camera_dose_avgs[::-1])

            # # Calculate percentage difference 
            # dose_percent_diff_high = dose_diff_high / dose_sensor_interp * 100


            ################################ Plots ##############################
            # plt.figure(1)
            # # Plot percentage difference 
            # plt.plot(self.sensor_y_axis, dose_percent_diff_low, color='green', label='Difference')
            # plt.legend(fontsize='large') 

            # plt.plot(ordered_camera_y_axis[::-1], dose_percent_diff_high, color='blue', label='Difference_2')
            # plt.legend(fontsize='large') 

            # plt.figure(2)
            # # Plot dose difference
            # plt.plot(self.sensor_y_axis, dose_diff_low, color='green', label='Difference')
            # plt.legend(fontsize='large') 

            # plt.plot(ordered_camera_y_axis[::-1], dose_diff_high, color='blue', label='Difference_2')
            # plt.legend(fontsize='large') 

            fig, ax1 = plt.subplots()

            # Plotting dose difference on the primary y-axis
            ax1.set_xlabel('Position (meters)',fontsize=16)
            ax1.set_ylabel('UV Dose $(W/m^2)$', color='tab:blue',fontsize=16)
            ax1.plot(self.sensor_y_axis, dose_diff_low, '--', color='tab:blue',label='Difference')
            ax1.tick_params(axis='y', labelcolor='tab:blue')

            # Create a grid for the primary y-axis
            ax1.grid(True, which="both", ls="-", c='.7')

            # Creating a secondary y-axis for the percentage
            ax2 = ax1.twinx()
            ax2.set_ylabel('Percentage (%)', color='tab:orange',fontsize=16)
            ax2.plot(self.sensor_y_axis, dose_percent_diff_low, color='tab:orange', label='Difference Percentage')
            ax2.tick_params(axis='y', labelcolor='tab:orange')

            # Since we have two axes, and we want a single legend, 
            # we need to gather the legend handles and labels from both axes.
            handles1, labels1 = ax1.get_legend_handles_labels()
            handles2, labels2 = ax2.get_legend_handles_labels()
            ax1.legend(handles1 + handles2, labels1 + labels2, loc='upper right',fontsize='large')


            plt.title('Dose Difference Between UV Sensor and Estimated Model',fontsize=20)

            plt.show()


if __name__ == '__main__':
    model = Plotter()
    model.plot_averages(difference=False)
    # model.plot_all_data()
    # model.plot_camera_data()
    # model.plot_sensor_data()
    # model.plot_camera_with_sensor_average()
