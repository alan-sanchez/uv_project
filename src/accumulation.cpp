/**
**  C++ version of python node: transform_accumulation_merge.py
**/

// // Import message types and other libraries
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <string>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>

#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "uv_project/uv_model.h"

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/transforms.h>
// #include <tf2_eigen/tf2_eigen.h>

using namespace std;
using namespace octomap;

class Accumulation {
    private:
    // // Initialize attributes
    ros::NodeHandle nh;
    ros::Subscriber combined_pcl2_sub;
    ros::Subscriber oct_center_pcl2_sub;
    ros::Subscriber command_sub;
    ros::Publisher MarkerArray_publisher;

    std_msgs::Header header;

    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerArray;

    tf::TransformListener listener;
    // tf2_ros::TransformListener listener2; 

    double resolution;
    double negative_z_arr[3];
    double magnitude_z_arr;
    double conical_bound;
    double required_dose;
    double prev_time;
    double uv_time_exposure;

    string command;

    public:
    Accumulation() {
        // // Initialize subscribers
        combined_pcl2_sub   = nh.subscribe("filtered_pcl2",               10, &Accumulation::callback_combined_pcl2,   this); 
        oct_center_pcl2_sub = nh.subscribe("octomap_point_cloud_centers", 10, &Accumulation::callback_oct_center_pcl2, this); 
        command_sub         = nh.subscribe("command",                     10, &Accumulation::callback_command,         this); 
        
        // // Initialize publisher
        MarkerArray_publisher = nh.advertise<visualization_msgs::Marker>("accumulation_map", 10);

        // // Intitialize OcTree class and acquire resolution
        ros::param::get("resolution", resolution);
        // OcTree tree(resolution); /*DOES NOT COMPILE*/

        // // Create array that points in the negative z direction from the `uv_light_link`
        negative_z_arr[0] = 0;
        negative_z_arr[1] = 0;
        negative_z_arr[2] = -0.3;

        // // Compute the magnitude of the the neagtive z direction array
        magnitude_z_arr = sqrt(pow(negative_z_arr[0], 2) + pow(negative_z_arr[1], 2) + pow(negative_z_arr[2], 2));

        // // Bound of conical angle (rads)
        conical_bound = 0.17;

        // // The required UV Dose for a UV rate constant of 0.0867 m^2/J
        // // at 99.9% disinfection rate is, 151.68 (J/m^2)
        required_dose = 151.68;

        // // Initialize `command` string message
        command = "None";
        // std_msgs::String command = "stop"; /* WHY DOES THIS NOT COMPILE?  */

        /*
        Created a header that calls a function that provides the irradiance for a given x value
        TODO: get model working in this class
        */

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

    }

    void callback_oct_center_pcl2(const sensor_msgs::PointCloud2& pcl2_msg){
        cout << "made it here" << endl;          
    }

    void callback_command(const std_msgs::String& str_msg){
        cout << "made it here" << endl;          
    }

    void callback_combined_pcl2(const sensor_msgs::PointCloud2& pcl_msg){
        double x = 1;
        //cout << pcl_msg << endl;
        // sensor_msgs::PointCloud2 pcl_out;
        // try
        // {
        //     listener.transformPointCloud("base_link", pcl_msg, pcl_out);
        // }
        // catch (tf::TransformExecptions& ex);
        // {
        //     ROS_ERROR("Transform error: %s", ex.what());
        //     return;
        // }
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

