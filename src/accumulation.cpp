/**
C++ version of python node: transform_accumulation_merge.py
**/

#include "uv_project/accumulation.h"

using namespace octomap;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace visualization_msgs;
using namespace geometry_msgs;

Accumulation::Accumulation() : tree(0.01){
    // // Initialize subscribers
    combined_pcl2_sub   = nh.subscribe("filtered_pcl2",               10, &Accumulation::callback_filtered_pcl2,   this);
    oct_center_pcl2_sub = nh.subscribe("octomap_point_cloud_centers", 10, &Accumulation::callback_oct_center_pcl2, this);
    command_sub         = nh.subscribe("command",                     10, &Accumulation::callback_command,         this);

    // // Initialize publisher
    // MarkerArray_publisher = nh.advertise<MarkerArray>("accumulation_map", 10);
    Marker_publisher = nh.advertise<Marker>("UV_dose_map", 10);

    // // Create array that points in the negative z direction from the `uv_light_link`
    negative_z_arr[0] =  0.0;
    negative_z_arr[1] =  0.0;
    negative_z_arr[2] = -0.3;

    // // Compute the magnitude of the the neagtive z direction array
    magnitude_z_arr = sqrt(pow(negative_z_arr[0], 2) + pow(negative_z_arr[1], 2) + pow(negative_z_arr[2], 2));

    // // Bound of conical angle (rads)
    conical_bound = 0.16;

    // // The required UV Dose for a UV rate constant of 0.0867 m^2/J
    // // at 99.9% disinfection rate is, 151.68 (J/m^2)
    required_dose = 151.68;

    // // Initialize `command` string message
    command.data = "None";

    // // Use current time for `prev_time` and set `uv_time_exposure` to zero
    prev_time = ros::Time::now().toSec();
    uv_time_exposure = 0;

    // // Initialize header
    header.frame_id = "/base_link";
    header.stamp = ros::Time::now();

    // // Initialize marker
    marker.header = header;
    marker.type = Marker::CUBE;
    marker.action = Marker::ADD;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.pose.orientation.w = 1.0;

    // // Initialize origin for octree
    origin = point3d(0,0,0);

    // // 
    dist_adjustment = 1.25;

    // // 
    num_cells_in_region = 0;

    // Prompt the user to select a region
    cout << "Select a region:" << endl;
    cout << "1. Sensor Array Region" << endl;
    cout << "2. Cone Region" << endl;
    cout << "3. Mug Region" << endl;

    // Read the user's choice
    cin >> region_choice;

    // Set the region bounds based on the user's choice
    if (region_choice == 1) {
        // Sensor Array region x and y bounds
        lower_x_bound = 0.72;
        upper_x_bound = 0.73;
        lower_y_bound = -0.52;
        upper_y_bound = 0.52;
        lower_z_bound = 0.30;
    } else if (region_choice == 2) {
        // Cone region x and y bounds
        lower_x_bound = 0.75;
        upper_x_bound = 0.85;
        lower_y_bound = -0.09;
        upper_y_bound = 0.1;
        lower_z_bound = 0.775;
    } else if (region_choice == 3) {
        // Mug region x and y bounds
        lower_x_bound = 0.80;
        upper_x_bound = 0.90;
        lower_y_bound = -0.09;
        upper_y_bound = 0.09;
        lower_z_bound = 0.775;
    } else {
        // Invalid choice, use default region
        cout << "Invalid choice. Using default region." << endl;
        lower_x_bound = 0.72;
        upper_x_bound = 0.73;
        lower_y_bound = -0.52;
        upper_y_bound = 0.52;
        lower_z_bound = 0.30;
    }

    min_key = tree.coordToKey(octomap::point3d(lower_x_bound, lower_y_bound, lower_z_bound));
    max_key = tree.coordToKey(octomap::point3d(upper_x_bound, upper_y_bound, 1.0));

    // // 
    ROS_INFO("Accumulation node is up and running.");
}


/*
A function that stores the PointCloud2 message
:param pcl2_msg: The PointCloud2 message
*/
void Accumulation::callback_oct_center_pcl2(const PointCloud2& pcl2_msg){
    oct_center_pcl2 = pcl2_msg;
}


/*
A function that transforms PointClouds to a desired transform frame
then creates an octree from those points.
:param str_msg: String message
*/
void Accumulation::callback_command(const String& str_msg){
    if (str_msg.data == "start") {
        // // 
        start_time = ros::Time::now();

        // // Set command string
        command.data = str_msg.data;
    }
    else if (str_msg.data == "stop") {
        // // 
        execution_time = ros::Time::now() - start_time;

        //
        command.data = str_msg.data;

        // // Open output file for writing
        ofstream output_file("output.csv");

        // Write headers
        output_file << "X_pos,Y_pos,Z_pos,UV_dose,Execution_time,Num_cells_in_region\n";

        // // Loop over dictionary keys and values
        for (auto const& data : acc_map_dict) {
            output_key = data.first;
            output_value = data.second;

            // // Write every key and value to file
            output_file << output_key[0] << "," << output_key[1] << "," << output_key[2] << "," << output_value << "," << execution_time << "," << num_cells_in_region << "\n";   
        }
        // // Close output file
        output_file.close();
    }
}


/*
Callback function that stores the PointCloud2 message of the combined
filtered image and depth map. This function als0 transforms the coordinates
from its original transform frame to the `base_link` and the `uv_light_link`
:param pcl2_msg: The PointCloud2 message
*/
void Accumulation::callback_filtered_pcl2(const PointCloud2& pcl2_msg){
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
            marker.action = Marker::DELETEALL;
            Marker_publisher.publish(marker);
            marker.action = Marker::ADD;
            ros::Duration(0.2).sleep();

            // // Transform PointCloud2 object to reference the `base_link`, then 
            // // convert to the octomap format and insert into the octree.
            temp_pcl2 = transform_PointCloud2(oct_center_pcl2,"base_link");
            pointCloud2ToOctomap(temp_pcl2, octomapCloud);
            tree.insertPointCloud(octomapCloud, origin);  

            // Loop through the keys within the region and count the cells
            for (int x = min_key[0]; x <= max_key[0]; x++) {
                for (int y = min_key[1]; y <= max_key[1]; y++) {
                    for (int z = min_key[2]; z <= max_key[2]; z++) {
                        octomap::OcTreeKey key(x, y, z);
                        if (tree.search(key) != NULL) {
                            num_cells_in_region++;
                        }
                    }
                }
            }
        } 
        cout<<num_cells_in_region<<endl;
        // // Create temporary dictionary
        map< vector<double>, vector<double> > temp_dict;
        vector<double> coord;

        // // Transform the `pcl2_msg` to reference the `base_link`. Then convert to a PointCloud object 
        baselink_pcl2 = transform_PointCloud2(pcl2_msg, "base_link");
        convertPointCloud2ToPointCloud(baselink_pcl2, baselink_pcl);

        // // Transform the `pcl2_msg` to reference the `uv_light_link`. Then convert to a PointCloud object
        uv_light_pcl2 = transform_PointCloud2(pcl2_msg, "uv_light_link");
        convertPointCloud2ToPointCloud(uv_light_pcl2, uv_light_pcl);
        
        // // 
        // omp_set_num_threads(5);
        // #pragma omp parallel for
        // // Use a for loop to check if the coordinates in the baselink_pcl is in the region
        for (size_t i = 0; i < uv_light_pcl.points.size(); ++i) {

            // //
            x_coord = baselink_pcl.points[i].x;
            y_coord = baselink_pcl.points[i].y;
            z_coord = baselink_pcl.points[i].z;

            // // // Check to see if point is in the predefined disinfeciton region
            // point_for_in_polygon_check = {baselink_pcl.points[i].x, baselink_pcl.points[i].y};
            // if (!in_poly.checkInside(polygon, sides, point_for_in_polygon_check)){
            
            if (x_coord >= lower_x_bound && 
                x_coord <= upper_x_bound && 
                y_coord >= lower_y_bound && 
                y_coord <= upper_y_bound &&
                z_coord >= lower_z_bound){
                // // Calculate the angle (radians) between the negative 
                // // z-axis vector and UV light x,y, and z coordinates
                ray_length = sqrt(pow(uv_light_pcl.points[i].x, 2) 
                                + pow(uv_light_pcl.points[i].y, 2) 
                                + pow(uv_light_pcl.points[i].z, 2));
                
                numerator = negative_z_arr[0] * uv_light_pcl.points[i].x
                          + negative_z_arr[1] * uv_light_pcl.points[i].y 
                          + negative_z_arr[2] * uv_light_pcl.points[i].z;

                denominator = magnitude_z_arr * ray_length;
                conical_angle = acos(numerator / denominator);

                if (conical_angle < conical_bound) {
                    point_in_conical_bound = point3d(baselink_pcl.points[i].x, 
                                                     baselink_pcl.points[i].y, 
                                                     baselink_pcl.points[i].z);
                                                    
                    // // Pull coordinates of KeyChecked cell and store them in a vector<double> type
                    tree.coordToKeyChecked(point_in_conical_bound, KeyChecked);
                    octomap_type_coord = tree.keyToCoord(KeyChecked);

                    // // 
                    node = tree.search(KeyChecked);
                    if (node != NULL) {
                        // // 
                        occ = tree.isNodeOccupied(node);

                        // // If point is octree, return the key, "KeyChecked"
                        if (occ) {
                            // // compute the UV dose for the `conical_angle`
                            radius = 0.3 * tan(conical_angle);
                            ir = irradiance.model(radius) * 10; // multiply by 10 to convert from mW/cm^2 to W/m^2
                            dist_ratio = pow(0.3, 2) / pow(ray_length*dist_adjustment, 2); // Inverse square law ratio
                            dose = dist_ratio * uv_time_exposure * ir;

                            // //
                            coord = {octomap_type_coord.x(), octomap_type_coord.y(), octomap_type_coord.z()};

                            // // store coordinates to temporary dictionary
                            if (temp_dict.find(coord)!=temp_dict.end()) {
                                temp_dict[coord].push_back(dose);
                            } else {
                                temp_dict[coord] = vector<double>{dose};
                            }
                        }
                    }
                }
            }
        } 
        
        // // Create marker array of cells
        for (auto const& data : temp_dict) {
            // // Pull key and value and determine the largest value
            key = data.first;
            value = data.second;
            max_dose_value = *(max_element(value.begin(), value.end()));

            // // Update accumulation map dictionary
            if (acc_map_dict.find(key)!=acc_map_dict.end()) {
                acc_map_dict[key] += max_dose_value;
            } else {
                acc_map_dict[key] = max_dose_value;
            }

            // // Update cube id dictionary
            if (cube_id_dict.find(key)!=cube_id_dict.end()) {
                marker.id = cube_id_dict[key];
            } else{
                cube_id_dict[key] = cube_id_dict.size();
                marker.id = cube_id_dict[key];
            }

            // // Update the color id based on UV values
            if (acc_map_dict[key] < (0.5 * required_dose)) {
                cube_color = define_color(1,0,0,0.5);
            } else if ((0.5 * required_dose) < acc_map_dict[key] && acc_map_dict[key] < (0.75 * required_dose)) {
                cube_color = define_color(1, 0.5, 0, 0.65);
            } else if ((0.75 * required_dose) < acc_map_dict[key] && acc_map_dict[key] < required_dose) {
                cube_color = define_color(1, 1, 0, 0.85);
            } else {
                cube_color = define_color(0, 1, 0, 1);
            }
            marker.color = cube_color;

            marker.pose.position.x = key[0];
            marker.pose.position.y = key[1];
            marker.pose.position.z = key[2];

            Marker_publisher.publish(marker);
        } 

        uv_time_exposure = ros::Time::now().toSec() - prev_time;
        prev_time = ros::Time::now().toSec();
    }            
}


/*
Function that transforms a PointCloud2 object to a target transform frame
:param pcl2_cloud: PointCloud2 message type
:param target_frame: String message type

:returns transfomred_pcl2: PountCloud2 message type
*/
PointCloud2 Accumulation::transform_PointCloud2( const PointCloud2& pcl2_cloud, const string& target_frame) {
    // // Loop until ROS is shutdown
    while (ros::ok()) {
        // pcl2_cloud.header.stamp = ros::Time(0); //::now().toSec();
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


/*
Function that defines the r, g, b, and a values for `ColorRGBA` object, cube_color
:param r: double message type
:param g: double message type
:param b: double message type
:param a: double message type
*/
std_msgs::ColorRGBA Accumulation::define_color(const double r, const double g, const double b, const double a) {
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}


int main (int argc, char **argv){
    // // Initialize the node
    ros::init(argc, argv, "accumulation");

    // // Instantiate Accumulation object
    Accumulation obj;

    // // Give control over to ROS
    ros::spin();
  	return 0;
}