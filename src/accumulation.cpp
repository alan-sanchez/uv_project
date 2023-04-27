/**
**  
**/

// // Import message types and other libraries
#include <ros/ros.h>
#include <tf/tf.h>
#include <octomap/octomap.h>
#include <cmath>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include "uv_project/uv_model.h"

using namespace std;


int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "vision_node");

  // Create a ROS node handle
  ros::NodeHandle nh;

  // // Call `model` function with argument value of 1
  // double irradiance = model(1);

  // // Print out irradiance value
  // cout << "Irradiance: " << irradiance <<endl;
}



