/**
C++ version of python node: transform_accumulation_merge.py
**/
// // Import message types and other libraries
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <set>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>

#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
// #include <tf2_eigen/tf2_eigen.h>

#include "uv_project/uv_model.h"
// #include "uv_project/in_polygon_check.h"

using namespace std;

class Accumulation {
    private:
    // // Initialize attributes
    ros::NodeHandle nh;
    ros::Subscriber combined_pcl2_sub;
    ros::Subscriber oct_center_pcl2_sub;
    ros::Subscriber command_sub;
    ros::Publisher MarkerArray_publisher;

    std_msgs::Header header;
    std_msgs::String command;
    sensor_msgs::PointCloud2 oct_center_pcl2;

    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerArray;

    tf::TransformListener listener;
    // tf2_ros::TransformListener listener2;

    octomap::Pointcloud octomapCloud;

    double resolution;
    double negative_z_arr[3];
    double magnitude_z_arr;
    double conical_bound;
    double required_dose;
    double prev_time;
    double uv_time_exposure;

    map<vector<double>, double> acc_map_dict;
    map<vector<double>, int> cube_id_dict;

    public:
    Accumulation() {
        // // Initialize subscribers
        combined_pcl2_sub   = nh.subscribe("filtered_pcl2",               10, &Accumulation::callback_filtered_pcl2,   this);
        oct_center_pcl2_sub = nh.subscribe("octomap_point_cloud_centers", 10, &Accumulation::callback_oct_center_pcl2, this);
        command_sub         = nh.subscribe("command",                     10, &Accumulation::callback_command,         this);

        // // Initialize publisher
        MarkerArray_publisher = nh.advertise<visualization_msgs::Marker>("accumulation_map", 10);

        // // Intitialize OcTree class and acquire resolution
        ros::param::get("resolution", resolution);
        octomap::OcTree tree(resolution);

        // // Create array that points in the negative z direction from the `uv_light_link`
        negative_z_arr[0] =  0.0;
        negative_z_arr[1] =  0.0;
        negative_z_arr[2] = -0.3;

        // // Compute the magnitude of the the neagtive z direction array
        magnitude_z_arr = sqrt(pow(negative_z_arr[0], 2) + pow(negative_z_arr[1], 2) + pow(negative_z_arr[2], 2));

        // // Bound of conical angle (rads)
        conical_bound = 0.17;

        // // The required UV Dose for a UV rate constant of 0.0867 m^2/J
        // // at 99.9% disinfection rate is, 151.68 (J/m^2)
        required_dose = 151.68;

        // // Initialize `command` string message
        command.data = "None";

        // // Use current time for `prev_time` and set `uv_time_exposure` to zero
        prev_time = ros::Time::now().toSec();
        uv_time_exposure = 0;

        // Initialize header
        header.frame_id = "/base_link";
        header.stamp = ros::Time::now();

        // Initialize marker
        marker.header = header;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = resolution;
        marker.scale.y = resolution;
        marker.scale.z = resolution;
        marker.pose.orientation.w = 1.0;

        // // Create sensor array region
        vector<vector<double>> region = {{0.72, 0.55}, {0.74, 0.55}, {0.74, -0.55}, {0.72, -0.55}};  // Sensor Array
        // vector<vector<double>> region = {{0.70, 0.07}, {0.90, 0.07}, {0.90, -0.113}, {0.70, -0.113}};  // Cone
        // vector<vector<double>> region = {{0.75, 0.05}, {0.90, 0.05}, {0.90, -0.09}, {0.75, -0.09}};  // Mug
    }


    /*
    A function that stores the PointCloud2 message
    :param pcl2_msg: The PointCloud2 message
    */
    void callback_oct_center_pcl2(const sensor_msgs::PointCloud2& pcl2_msg){
        oct_center_pcl2 = pcl2_msg;
    }


    /*
    A function that transforms PointClouds to a desired transform frame
    then creates an octree from those points.
    :param str_msg: String message
    */
    void callback_command(const std_msgs::String& str_msg){
        if (str_msg.data == "start") {
            // // Clear previous octree, markers, and dictrionaries
            // // Intitialize OcTree class and acquire resolution
            tree.clear(); 
            acc_map_dict.clear();
            cube_id_dict.clear();
            marker.action = visualization_msgs::Marker::DELETEALL;
            markerArray.markers.push_back(marker);
            MarkerArray_publisher.publish(markerArray);
            marker.action = visualization_msgs::Marker::ADD;
            ros::Duration(0.2).sleep();

            // Transform PointCloud2 object to reference the `base_link`
            string target_frame = "base_link";
            sensor_msgs::PointCloud2 temp_pcl2;
            temp_pcl2 = transform_pointcloud(oct_center_pcl2,target_frame);
            
            // // create an `octomap::Pointcloud` object
            octomap::Pointcloud octomapCloud;

            // // convert the Pointcloud2 message type to the octomap format
            octomap::pointCloud2ToOctomap(oct_center_pcl2, octomapCloud);

            // // Origin at base_link
            octomap::point3d origin(0, 0, 0);
            
            // // Insert poincloud into octree
            tree.insertPointCloud(octomapCloud, origin);

            // Set command string
            command.data = str_msg.data;
        }
        else if (str_msg.data == "stop") {
            command.data = str_msg.data;

            // // Open output file for writing
            // std::ofstream output_file("output.csv");

            // // Loop over dictionary keys and values
            // for (auto const& [key, val] : acc_map_dict) {
            //     // Write every key and value to file
            //     std::vector<std::string> row = {std::to_string(key[0]), std::to_string(key[1]), std::to_string(key[2]), std::to_string(val)};
            //     for (auto const& field : row) {
            //         output_file << field << ",";
            //     }
            //     output_file << std::endl;
            // }
            // // Close output file
            // output_file.close();
        }
    }


    /*
    Callback function that stores the PointCloud2 message of the combined
    filtered image and depth map. This function als transforms the coordinates
    from its original transform frame to the `base_link` and `uv_light_link`
    :param pcl2_msg: The PointCloud2 message
    */
    void callback_filtered_pcl2(const sensor_msgs::PointCloud2& pcl2_msg){
        if (command.data == "start") {

            // //  This resets the `prev_time` variable to current time and sets `uv_time_exposure` to zero
            // //  after a trajectory execution is complete. This is because there is a 2 second wait 
            // //  time before the user can command another trajectory
            if ((ros::Time::now().toSec() - prev_time) > 2.0) {
                prev_time = ros::Time::now().toSec();
                uv_time_exposure = 0;
            } 

            // // Create temporary dictionary
            map<vector<double>, double> temp_dict;

            // // Transform the `pcl2_msg` to reference the `base_link` then create a 
            // // PointCloud object 
            sensor_msgs::PointCloud2 baselink_pcl2 = transform_pointcloud(pcl2_msg, "base_link");
            sensor_msgs::PointCloud baselink_pcl;
            sensor_msgs::convertPointCloud2ToPointCloud(baselink_pcl2, baselink_pcl);

            // //  Transform the `pcl2_msg` to reference the `uv_light_link` then create a 
            // // PointCloud object
            sensor_msgs::PointCloud2 uv_light_pcl2 = transform_pointcloud(pcl2_msg, "uv_light_link");
            sensor_msgs::PointCloud uv_light_pcl;
            sensor_msgs::convertPointCloud2ToPointCloud(uv_light_pcl2, uv_light_pcl);

            // // // Use a for loop to check if the coordinates in the baselink_pcl is in the region
            for (size_t i = 0; i < uv_light_pcl.points.size(); ++i) {
                const auto& uv_light_coord = uv_light_pcl.points[i];
                const auto& base_coord = baselink_pcl.points[i];

                /*
                Include in_polygon_check here
                */

                // Calculate the angle (radians) between the z-axis vector and 
                // uv flashlight point coordinates, `uv_light_coord`
                double ray_length = sqrt(pow(uv_light_coord.x, 2) + pow(uv_light_coord.y, 2) + pow(uv_light_coord.z, 2));
                double numerator = negative_z_arr[0] * uv_light_coord.x + negative_z_arr[1] * uv_light_coord.y + negative_z_arr[2] * uv_light_coord.z;
                double denominator = magnitude_z_arr * ray_length;
                double rad = acos(numerator / denominator);

                if (rad < conical_bound) {
                    octomap::OcTreeKey key;
                    octomap::point3d pnt(base_coord.x, base_coord.y, base_coord.z);
                    bool chk = tree.coordToKeyChecked(pnt, key)
                }
            }      
            
                  
        }
      
    }


    sensor_msgs::PointCloud2 transform_pointcloud( const sensor_msgs::PointCloud2& pcl2_cloud, const std::string& target_frame) {
        // // Loop until ROS is shutdown
        while (ros::ok()) {
            try {
                // // Initialize a new pointcloud for the transformed output
                sensor_msgs::PointCloud2 new_cloud;

                // // Use pcl_ros library to transform pointcloud to target frame
                pcl_ros::transformPointCloud(target_frame, pcl2_cloud, new_cloud, listener);
                return new_cloud;
            }
            catch (tf::TransformException& ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(0.1).sleep();
            }
        }
    }
};

   


int main (int argc, char **argv)
{
    // // Initialize the node
    ros::init(argc, argv, "accumulation");
    // ros::NodeHandle nh;

    // // Instantiate TransformPCL object
    Accumulation obj;

    // // Give control over to ROS
    ros::spin();
  	return 0;

    // // Call `model` function with argument value of 1
    // double irradiance = model(1);

    // // Print out irradiance value
    // cout << "Irradiance: " << irradiance <<endl;
}
