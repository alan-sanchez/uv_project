// /**
// **  
// **/

// Import message types and other libraries
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <geometry_msgs/Point32.h>

#include <std_msgs/String.h>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/transforms.h>
// #include <tf2_eigen/tf2_eigen.h>

using namespace std;

class TransformPCL {
    private:
    ros::NodeHandle nh;
    ros::Subscriber combined_pcl2_sub;
    ros::Subscriber oct_center_pcl2_sub;
    ros::Subscriber command_sub;
    tf::TransformListener listener;
    // tf2_ros::TransformListener listener_;
  
    public:
    TransformPCL() {
        // Initialize subscribers
        combined_pcl2_sub   = nh.subscribe("filtered_pcl2",               10, &TransformPCL::callback_combined_pcl2,   this); 
        oct_center_pcl2_sub = nh.subscribe("octomap_point_cloud_centers", 10, &TransformPCL::callback_oct_center_pcl2, this); 
        command_sub         = nh.subscribe("command",                     10, &TransformPCL::callback_command,         this); 
        
        // Initialize publisher

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
    ros::init(argc, argv, "transform_pcl_cpp");
    // ros::NodeHandle nh;

    // Instantiate TransformPCL object
    TransformPCL obj;

    // Give control over to ROS
    ros::spin();
  	return 0;
}


