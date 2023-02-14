#!/usr/bin/env python3

## Import modules
import rospy
import numpy as np
import octomap
from octomap_msgs.msg import Octomap

## Import message types and other pyton libraries
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import String
from geometry_msgs.msg import Point32

class AccumulationMap:
    """
    A class that subscribes to the UV direction vectors and publishes a matrix
    that represents a UV accumulation depth map.
    """
    def __init__(self):
        """
        Function that initializes the subscriber, publisher, and other variables.
        :param self: The self reference.
        """
        ## Initialize Subscribers
        self.pointcloud_sub  = rospy.Subscriber('/combined_filtered_image_and_depthmap', PointCloud2, self.callback_pointcloud2)
        # self.oct_sub         = rospy.Subscriber('/octomap_binary',                       Octomap,     self.callback)

        ## Initialize Publishers
        self.oct_pub  = rospy.Publisher('/new_oct_binary', Octomap, queue_size=10)

        ## Initialize OcTree function with a resolution of 0.05 meters
        self.resolution = 0.05
        self.octree = octomap.OcTree(self.resolution)

        ## Initialize self.pointcloud variable
        self.temp_pc2 = None


    def callback_pointcloud2(self, msg):
        """
        
        """
        self.temp_pc2 = msg

   
        pcl_cloud = PointCloud()

        ## For loop to extract pointcloud2 data into a list of x,y,z, and
        ## store it in a pointcloud message (pcl_cloud)
        for data in pc2.read_points(self.temp_pc2, skip_nans=True):
            pcl_cloud.points.append(Point32(data[0],data[1],data[2]))

        ## Parse the filtered cloud's points as a np.array. This action is required
        ## to pass as an agrument in the insertPointCloud() function.
        self.pointcloud = np.empty(shape=[len(pcl_cloud.points),3])
        self.temp_acc_map = np.empty(shape=[len(pcl_cloud.points),4])
        for i in range(len(pcl_cloud.points)):
            self.pointcloud[i] = [pcl_cloud.points[i].x,
                                  pcl_cloud.points[i].y,
                                  pcl_cloud.points[i].z]


        ## Insert a 3D scaninto the the tree
        self.octree.insertPointCloud(pointcloud = self.pointcloud, origin = np.array([0, 0, 0], dtype=float))
        # print(type(self.octree))
        test_file = self.octree.writeBinary()
        # print(test_file)

        test_file_2 = self.octree.write()
        print(test_file_2)

        # a = self.octree

        # oct_msg.data=test_file
        # self.oct_pub.publish(oct_msg)


if __name__=="__main__":
    ## Initialize accumulation_map node
    rospy.init_node('accumulation_map',anonymous=True)

    ## Instantiate the AccumulationMap class
    AccumulationMap()
    rospy.spin()
