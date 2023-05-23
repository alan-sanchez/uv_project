#ifndef ACCUMULATION_H
#define ACCUMULATION_H

#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <map>
#include <set>
#include <omp.h>

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
        ros::Publisher Marker_publisher;

        std_msgs::Header header;
        std_msgs::String command;
        std_msgs::ColorRGBA cube_color;
        std_msgs::ColorRGBA color;

        sensor_msgs::PointCloud2 oct_center_pcl2;
        sensor_msgs::PointCloud2 temp_pcl2;
        sensor_msgs::PointCloud2 baselink_pcl2;
        sensor_msgs::PointCloud2 uv_light_pcl2;
        sensor_msgs::PointCloud2 transformed_pcl2;
        sensor_msgs::PointCloud transformed_pcl;
        sensor_msgs::PointCloud baselink_pcl;
        sensor_msgs::PointCloud uv_light_pcl;

        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray markerArray;

        tf::TransformListener listener;

        octomap::Pointcloud octomapCloud;
        octomap::OcTree tree;
        octomap::OcTreeKey KeyChecked;
        octomap::point3d origin;
        octomap::point3d point_in_conical_bound;
        octomap::point3d octomap_type_coord;
        octomap::OcTreeNode* node;

        uv_model irradiance;
        Check in_poly;

        int sides;
        bool occ;
        double negative_z_arr[3];
        double magnitude_z_arr;
        double conical_bound;
        double required_dose;
        double prev_time;
        double uv_time_exposure;
        double ray_length;
        double numerator;
        double denominator;
        double rad;
        double radius;
        double ir;
        double dist_ratio;
        double dose;
        double max_dose_value;
        double r; double g; double b; double a;
        double output_value;
        vector<double> output_key;
        vector<double> value;
        vector<double> key;
        map<vector<double>, double> acc_map_dict;
        map<vector<double>, int> cube_id_dict;

        // // Create polygon region
        Check::Point polygon[4] =  {{0.72, 0.55}, {0.74, 0.55}, {0.74, -0.55}, {0.72, -0.55}};  // Sensor Array
        //Check::Point polygon[] =  {{0.70, 0.07}, {0.90, 0.07}, {0.90, -0.113}, {0.70, -0.113}};  // Cone
        //Check::Point polygon[] =  {{0.75, 0.05}, {0.90, 0.05}, {0.90, -0.09}, {0.75, -0.09}};  // Mug

        Check::Point point_for_in_polygon_check;

    public:
        Accumulation();

        void callback_oct_center_pcl2(const sensor_msgs::PointCloud2& pcl2_msg);

        void callback_command(const std_msgs::String& str_msg);

        void callback_filtered_pcl2(const sensor_msgs::PointCloud2& pcl2_msg);

        std_msgs::ColorRGBA define_color(const double r, const double g, const double b, const double a);

        sensor_msgs::PointCloud2 transform_PointCloud2( const sensor_msgs::PointCloud2& pcl2_cloud, const std::string& target_frame);
};
#endif