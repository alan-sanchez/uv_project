#ifndef ACCUMULATION_H
#define ACCUMULATION_H

#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <map>
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
#include "uv_project/in_polygon_check.h"

using namespace std;


class Accumulation {
    private:
        // // Initialize variables, objects, and functions
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
        octomap::OcTree tree;
        // // Intitialize OcTree class and acquire resolution
        // ros::param::get("resolution", resolution);
        // double res = 0.01;
        // octomap::OcTree tree(res);



        double resolution;
        double negative_z_arr[3];
        double magnitude_z_arr;
        double conical_bound;
        double required_dose;
        double prev_time;
        double uv_time_exposure;

        map<vector<double>, double> acc_map_dict;
        map<vector<double>, int> cube_id_dict;

        // double radius = 1.0;
        // double irradiance = uv_model(radius);

    public:
        Accumulation();
        // 
        void callback_oct_center_pcl2(const sensor_msgs::PointCloud2& pcl2_msg);

        //
        void callback_command(const std_msgs::String& str_msg);

        //
        void callback_filtered_pcl2(const sensor_msgs::PointCloud2& pcl2_msg);

        //
        sensor_msgs::PointCloud2 transform_pointcloud( const sensor_msgs::PointCloud2& pcl2_cloud, const std::string& target_frame);

};

#endif