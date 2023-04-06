#!/usr/bin/env python3

## Import modules
import rospy
import numpy as np
import math
import octomap
import message_filters
import csv

## Import message types and other python libraries
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from best_fit import fit
from shapely import geometry


class AccumulationMap(object):
    """
    A class that builds an accumulation UV dose map.
    """
    def __init__(self):
        """
        A function that initialises the subscriber, publisher, and other variables.
        :param self: The self reference.
        """
        ## Initialize Subscribers
        self.oct_center_pcl2_sub = rospy.Subscriber('/octomap_centers_pcl', PointCloud,  self.callback_oct_center_pcl)
        self.start_sub           = rospy.Subscriber('/command',             String,      self.callback_command)
        
        self.transformed_pcl_sub = message_filters.Subscriber('/gripper_reference_pcl',  PointCloud)
        self.combined_pcl_sub    = message_filters.Subscriber('/baselink_reference_pcl', PointCloud)
        sync = message_filters.ApproximateTimeSynchronizer([self.transformed_pcl_sub,
                                                            self.combined_pcl_sub],
                                                            queue_size=5,
                                                            slop=0.1)
        sync.registerCallback(self.callback_sync)

        ## Initialize Publisher
        self.MarkerArray_publisher = rospy.Publisher('/accumulation_map', MarkerArray, queue_size=10)
        
        ## Initialize self.cloud for data storage in pointcloud_data callback function
        self.oct_center_pcl2 = None

        ## Initialize OcTree function with a resolution of 0.05 meters
        self.resolution = 0.05
        self.octree = octomap.OcTree(self.resolution)

        ## vector the points straight down in the z direction from `uv_light_link` tf
        self.grip_vec = [0,0,-.3]
        self.mag_grip_vec = np.linalg.norm(self.grip_vec)

        ## outerbound of conical angle
        self.bound = 0.17 

        ## The required UV Dose for a UV rate constant of 0.0867 m^2/J
        ## at 99% disinfection rate is, 53.10 (J/m^2)
        self.required_dose = 53.10

        ## Command that begins and stops accumulation map and markers
        self.command = None

        ## Bring in the UV light source model as an equation
        self.eqn_model = fit(16, plotter = False) 

        ## Set a previous time as now and time exposure as zero
        self.prev_time = rospy.get_time()
        self.time_exposure = 0

        # Initiate dictionaries
        self.acc_map_dict = dict()
        self.cube_id_dict = dict()

        ## Initialize header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        ## Initialize Marker
        self.marker = Marker()
        self.marker.header = self.header
        self.marker.type = Marker.CUBE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = self.resolution
        self.marker.scale.y = self.resolution
        self.marker.scale.z = self.resolution
        self.marker.pose.orientation.w = 1.0

        ## Initialize MarkerArray
        self.markerArray = MarkerArray()

        ## Create sensor array region 
        self.region = [[0.71, 0.55], [0.76, 0.55], [0.76, -0.55], [0.71, -0.55]]
        self.line = geometry.LineString(self.region)
        self.polygon = geometry.Polygon(self.line)


    def callback_command(self, str_msg):
        """
        Function that stores the filtered point cloud and create new octree.
        :param self: The self reference.
        :param msg: The PointCloud message type.
        """
        rospy.sleep(0.1)
        if str_msg.data == "start":
            ## Clear previous octree, markers, and dictrionaries
            self.octree.clear()
            self.acc_map_dict = dict()
            self.cube_id_dict = dict()
            self.marker.action = Marker.DELETEALL
            self.markerArray.markers.append(self.marker)
            self.MarkerArray_publisher.publish(self.markerArray)
            self.marker.action = Marker.ADD
            rospy.sleep(0.2)
            
            ## Parse the filtered cloud's points as a np.array. This is required
            ## to pass as an agrument in the `insertPointCloud()` method.
            self.pointcloud = np.empty(shape=[len(self.oct_center_pcl.points),3])
            
            for i in range(len(self.oct_center_pcl.points)):
                self.pointcloud[i] = [self.oct_center_pcl.points[i].x,
                                      self.oct_center_pcl.points[i].y,
                                      self.oct_center_pcl.points[i].z]

            ## Insert a 3D scan into the the tree
            self.octree.insertPointCloud(pointcloud = self.pointcloud, origin = np.array([0, 0, 0], dtype=float)) # self.octree.writeBinary(b"before.bt")

            ## Set command
            self.command = str_msg.data

        elif str_msg.data == "stop": 
            ## Set command
            self.command = str_msg.data
            # print(self.acc_map_dict)
            ## open file for writing, "w" is writing
            w = csv.writer(open("output.csv", "w"))

            ## loop over dictionary keys and values
            for key, val in self.acc_map_dict.items():
                key_list = list(key)
                ## write every key and value to file
                w.writerow([key[0],key[1],key[2],val])


    def callback_oct_center_pcl(self,pcl_msg):
        """
        A callback function that 
        :param self: The self reference.
        :param pcl_msg: A PointCloud message.
        """
        self.oct_center_pcl = pcl_msg
    

    def callback_sync(self, uv_light_pcl, baselink_pcl):
        """
        Callback function that transforms gripper camera to reference the `base_link` 
        and `uv_light_link`.
        :param self: The self reference.
        :param uv_light_pcl: A PointCloud message referencing the uv_light_link.
        :param baselink_pcl: A PointCloud message referencing the base_link.
        """
        ## This resets the prev_time variable to None after a trajectory execution
        ## is complete. This is because there is a 2 second wait time before the user
        ## can command another trajectory
        if (rospy.get_time() - self.prev_time) > 2.0:
            self.prev_time = rospy.get_time()
            self.time_exposure = 0

        ## Create temporary empty lists and execute for loop to temporary uv dose map
        temp_dict = dict()

        for uv_light_coord, base_coord in zip(uv_light_pcl.points, baselink_pcl.points):
            ## Check if the base_coord is in the defined region    
            point = geometry.Point(base_coord.x,base_coord.y)
            if self.polygon.contains(point) == False:
                continue

            ## Calculate the angle (radians) between the z-axis vector and 
            ## uv flashlight point coordinates, `uv_light_coord`
            ray_length = np.linalg.norm([uv_light_coord.x,uv_light_coord.y,uv_light_coord.z])        
            numerator = np.dot(self.grip_vec, [uv_light_coord.x,uv_light_coord.y,uv_light_coord.z])
            denominator = self.mag_grip_vec * ray_length
            rad = np.arccos(numerator/denominator)

            # print(rad)
           
            if rad < self.bound:
                chk, key = self.octree.coordToKeyChecked(np.array([base_coord.x, base_coord.y, base_coord.z]))
                # print(chk)
                if chk:
                    ## compute the UV dose for the conical `rad` value
                    radius = 0.3 * math.tan(rad)
                    ir = self.eqn_model(radius) * 10 # multiply by 10 to convert from mW/cm^2 to W/m^2
                    dist_ratio = (0.3**2)/(ray_length**2) # Inverse sqaure law ratio
                    dose = dist_ratio * self.time_exposure * ir

                    ## Pull coordinates of cell key
                    pos = tuple(self.octree.keyToCoord(key))
                    
                    if pos in temp_dict:
                        temp_dict[pos].append(dose)     
                    else:
                        temp_dict[pos] = [dose,]

        ## Create marker array of cells
        for pose_key in temp_dict:
            dose_value = sum(temp_dict[pose_key])/len(temp_dict[pose_key])

            ## Update accumulation map dictionary
            if pose_key in self.acc_map_dict:
                self.acc_map_dict[pose_key] += dose_value
            else:
                self.acc_map_dict[pose_key] = dose_value

            ## Update cube id dictionary
            if pose_key in self.cube_id_dict:
                self.marker.id = self.cube_id_dict[pose_key]
            else:
                self.cube_id_dict[pose_key] = len(self.cube_id_dict)

                self.marker.id = self.cube_id_dict[pose_key]
            
            ## Update the color id based on UV values
            if self.acc_map_dict[pose_key] < 0.25 * self.required_dose:
                self.marker.color = ColorRGBA(1,   0,  0, 0.25)
            elif self.acc_map_dict[pose_key] < 0.5 * self.required_dose:
                self.marker.color = ColorRGBA(1, 0.5,  0, 0.50)
            elif self.acc_map_dict[pose_key] < 0.75 * self.required_dose:
                self.marker.color = ColorRGBA(1, 1,  0, 0.75)
            else:
                self.marker.color = ColorRGBA(  0, 1,  0, 1)

            ## Set point to marker
            self.marker.points = [Point(pose_key[0], pose_key[1], pose_key[2]),]

            ## Append and publish 
            self.markerArray.markers.append(self.marker)
            self.MarkerArray_publisher.publish(self.markerArray)

        ## Set new prev_time as current time before the beginning of next loop iteration
        self.time_exposure = rospy.get_time() - self.prev_time
        self.prev_time = rospy.get_time()

if __name__=="__main__":
    ## Initialize accumulation_map node
    rospy.init_node('accumulation_map',anonymous=True)

    ## Instantiate the `AccumulationMap()` class
    AccumulationMap()
    rospy.spin()