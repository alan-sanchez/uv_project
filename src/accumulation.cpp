/**
C++ version of python node: transform_accumulation_merge.py
**/

#include "uv_project/accumulation.h"

using namespace octomap;
using namespace sensor_msgs;

Accumulation::Accumulation() : tree(0.01){
    // // Initialize subscribers
    combined_pcl2_sub   = nh.subscribe("filtered_pcl2",               10, &Accumulation::callback_filtered_pcl2,   this);
    oct_center_pcl2_sub = nh.subscribe("octomap_point_cloud_centers", 10, &Accumulation::callback_oct_center_pcl2, this);
    command_sub         = nh.subscribe("command",                     10, &Accumulation::callback_command,         this);

    // // Initialize publisher
    MarkerArray_publisher = nh.advertise<visualization_msgs::Marker>("accumulation_map", 10);

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

    origin = point3d(0,0,0);

    // // 
    sides = sizeof(polygon);
}


/*
A function that stores the PointCloud2 message
:param pcl2_msg: The PointCloud2 message
*/
void Accumulation::callback_oct_center_pcl2(const sensor_msgs::PointCloud2& pcl2_msg){
    oct_center_pcl2 = pcl2_msg;
}


/*
A function that transforms PointClouds to a desired transform frame
then creates an octree from those points.
:param str_msg: String message
*/
void Accumulation::callback_command(const std_msgs::String& str_msg){
    if (str_msg.data == "start") {
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
filtered image and depth map. This function als0 transforms the coordinates
from its original transform frame to the `base_link` and the `uv_light_link`
:param pcl2_msg: The PointCloud2 message
*/
void Accumulation::callback_filtered_pcl2(const sensor_msgs::PointCloud2& pcl2_msg){
    if (command.data == "start") {
        // // This resets the all dictionaries, variables, and OcTree between
        // // each joint trajectory execution 
        if ((ros::Time::now().toSec() - prev_time) > 2.0) {
            prev_time = ros::Time::now().toSec();
            uv_time_exposure = 0;

            // // Clear previous octree, markers, and dictionaries
            tree.clear(); 
            acc_map_dict.clear();
            cube_id_dict.clear();
            marker.action = visualization_msgs::Marker::DELETEALL;
            markerArray.markers.push_back(marker);
            MarkerArray_publisher.publish(markerArray);
            marker.action = visualization_msgs::Marker::ADD;
            ros::Duration(0.2).sleep();

            // // Transform PointCloud2 object to reference the `base_link`, then 
            // // convert to the octomap format and insert into the octree.
            temp_pcl2 = transform_pointcloud(oct_center_pcl2,"base_link");
            pointCloud2ToOctomap(oct_center_pcl2, octomapCloud);
            tree.insertPointCloud(octomapCloud, origin);  
        } 
        // // Create temporary dictionary
        map<point3d, vector<double>> temp_dict;

        // // Transform the `pcl2_msg` to reference the `base_link`. Then convert to a PointCloud object 
        baselink_pcl2 = transform_pointcloud(pcl2_msg, "base_link");
        convertPointCloud2ToPointCloud(baselink_pcl2, baselink_pcl);

        // // Transform the `pcl2_msg` to reference the `uv_light_link`. Then convert to a PointCloud object
        uv_light_pcl2 = transform_pointcloud(pcl2_msg, "uv_light_link");
        convertPointCloud2ToPointCloud(uv_light_pcl2, uv_light_pcl);

        // // Use a for loop to check if the coordinates in the baselink_pcl is in the region
        for (size_t i = 0; i < uv_light_pcl.points.size(); ++i) {
            // cout << uv_light_pcl.points[i].x<< endl;
            // // 
            point_for_in_polygon_check = {baselink_pcl.points[i].x, baselink_pcl.points[i].y};
            if (!in_poly.checkInside(polygon, sides, point_for_in_polygon_check)){
                continue;
            }
            
            // // Calculate the angle (radians) between the negative 
            // // z-axis vector and UV light x,y, and z coordinates
            ray_length = sqrt(pow(uv_light_pcl.points[i].x, 2) 
                            + pow(uv_light_pcl.points[i].y, 2) 
                            + pow(uv_light_pcl.points[i].z, 2));
            
            numerator = negative_z_arr[0] * uv_light_pcl.points[i].x
                      + negative_z_arr[1] * uv_light_pcl.points[i].y 
                      + negative_z_arr[2] * uv_light_pcl.points[i].z;

            denominator = magnitude_z_arr * ray_length;
            rad = acos(numerator / denominator);

            // // 
            if (rad < conical_bound) {
                point_in_conical_bound = point3d(baselink_pcl.points[i].x, 
                                                 baselink_pcl.points[i].y, 
                                                 baselink_pcl.points[i].z);

                if (tree.coordToKeyChecked(point_in_conical_bound, key)) {
                    // // compute the UV dose for the conical `rad` value
                    radius = 0.3 * tan(rad);
                    ir = irradiance.model(radius) * 10; // multiply by 10 to convert from mW/cm^2 to W/m^2
                    dist_ratio = pow(0.3, 2) / pow(ray_length, 2); // Inverse square law ratio
                    dose = dist_ratio * uv_time_exposure * ir;

                    // // Pull coordinates of cell key
                    key_coord = tree.keyToCoord(key);

                    // if (temp_dict.find(key_coord)!=temp_dict.end()) {
                    //     cout << "made it here" << endl;
                        // temp_dict[key_coord].push_back(dose);
                        // return 1;
                    // }
                    //  else {
                    //     temp_dict[key_coord] = vector<double>{dose};
                    // }
                }
            }
        } 
    }            
}


sensor_msgs::PointCloud2 Accumulation::transform_pointcloud( const sensor_msgs::PointCloud2& pcl2_cloud, const std::string& target_frame) {
    // // Loop until ROS is shutdown
    while (ros::ok()) {
        try {          
            // // Use pcl_ros library to transform pointcloud to target frame
            pcl_ros::transformPointCloud(target_frame, pcl2_cloud, transformed_pcl2, listener);
            return transformed_pcl2;
        }
        catch (tf::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
        }
    }
}



int main (int argc, char **argv){
    // // Initialize the node
    ros::init(argc, argv, "accumulation");
    // ros::NodeHandle nh;

    // // Instantiate TransformPCL object
    Accumulation obj;

    // // Give control over to ROS
    ros::spin();
  	return 0;
}
