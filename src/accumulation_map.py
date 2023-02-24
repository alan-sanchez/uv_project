#!/usr/bin/env python3

# Import modules
import rospy
import numpy as np
import math
import octomap
import message_filters
import sensor_msgs.point_cloud2 as pc2

# Import message types and other python libraries
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Header, String

class AccumulationMap(object):
    """
    A class that publishes the direction and irridance values of the UV vectors
    whose origins are from the end effector.
    """
    def __init__(self):
        """
        A function that initialises  the subscriber, publisher, and other variables.
        :param self: The self reference.
        """
        ## Initialize Subscribers
        self.oct_center_pcl2_sub = rospy.Subscriber('/octomap_pcl_centers', PointCloud,  self.callback_oct_center_pcl)
        self.start_sub           = rospy.Subscriber('/command',             String,      self.callback_command)

        ## Initialize subscribers
        self.transformed_pcl_sub = message_filters.Subscriber('/transformed_cloud',    PointCloud)
        self.combined_pcl_sub    = message_filters.Subscriber('/combined_point_cloud', PointCloud)

        sync = message_filters.ApproximateTimeSynchronizer([self.transformed_pcl_sub,
                                                            self.combined_pcl_sub],
                                                            queue_size=5,
                                                            slop=0.1)
        sync.registerCallback(self.callback_sync)
        
        ## Initialize self.cloud for data storage in pointcloud_data callback function
        self.oct_center_pcl2 = None

        ## Initialize OcTree function with a resolution of 0.05 meters
        self.resolution = 0.02
        self.octree = octomap.OcTree(self.resolution)

        ## The required UV Dose for a UV rate constant of 0.0867 m^2/J is 132.8 (J/m^2)
        self.required_dose = -132.8

        ## vector
        self.a = [0,0,-.3]
        self.mag_a = np.linalg.norm(self.a)

        ## outerbound of conical angle
        self.bound = 0.6#0.17 

        ## 
        self.command = None

    def callback_command(self, str_msg):
        """
        Function that stores the filtered point cloud and create new octree for
        castRay calculations.
        :param self: The self reference.
        :param msg: The PointCloud message type.
        """
        if str_msg.data == "start":
            self.octree.clear()
            rospy.sleep(0.2)
            ## Parse the filtered cloud's points as a np.array. This action is required
            ## to pass as an agrument in the insertPointCloud() function.
            self.pointcloud = np.empty(shape=[len(self.oct_center_pcl.points),3])
            self.temp_acc_map = np.empty(shape=[len(self.oct_center_pcl.points),4])
            for i in range(len(self.oct_center_pcl.points)):
                self.pointcloud[i] = [self.oct_center_pcl.points[i].x,
                                    self.oct_center_pcl.points[i].y,
                                    self.oct_center_pcl.points[i].z]

                self.temp_acc_map[i] = [self.oct_center_pcl.points[i].x,
                                        self.oct_center_pcl.points[i].y,
                                        self.oct_center_pcl.points[i].z,
                                        self.required_dose]

            ## Insert a 3D scaninto the the tree
            self.octree.insertPointCloud(pointcloud = self.pointcloud, origin = np.array([0, 0, 0], dtype=float))
            self.octree.begin_tree()
            # message = bytes("test.bt", 'utf-8')
            # self.octree.writeBinary(b"test_map.bt")
            ##
            self.command = str_msg.data

    def callback_oct_center_pcl(self,pcl_msg):
        """
        A callback function that 
        :param self: The self reference.
        :param
        """
        self.oct_center_pcl = pcl_msg
    
    def callback_sync(self, transformed_pcl, combined_pcl):
        """
        Callback function that
        :param self: The self reference.
        :param polygon: The PolygonStamped message.

        :publishes self.filtered_cloud: Filtered PointCloud message.
        """
        index = 0
        for tip, loc in zip(transformed_pcl.points, combined_pcl.points):
            numerator = np.dot(self.a, [tip.x,tip.y,tip.z])
            denominator = self.mag_a * np.linalg.norm([tip.x,tip.y,tip.z])
            rad = np.arccos(numerator/denominator)
            # print("conical angle in radians: " + str(rad))
            if rad < self.bound:
                print(loc.x, loc.y, loc.z)
                # print(self.octree.inBBX(np.array([loc.x, loc.y, loc.z])))
                chk, res = self.octree.coordToKeyChecked(np.array([loc.x, loc.y, loc.z]))
                # print(chk)
                # print(res)
                pos = self.octree.keyToCoord(res)
                print(pos)



            index+=1
   
if __name__=="__main__":
    ## Initialize irradiance_vectors node
    rospy.init_node('accumulation_map',anonymous=True)

    ## Instantiate the IrradianceVectors class
    AccumulationMap()
    rospy.spin()