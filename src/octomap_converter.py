#!/usr/bin/env python3

## Import modules
import rospy
import numpy as np
import octomap
import message_filters
import sensor_msgs.point_cloud2 as pc2

## Import message types and other pyton libraries
from sensor_msgs.msg import PointCloud2, PointCloud
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from octomap_msgs.msg import Octomap, OctomapWithPose


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
        ## Initialize subscribers and time synchronizer
        self.pcl2_sub = message_filters.Subscriber('/combined_filtered_image_and_depthmap', PointCloud2)
        self.oct_sub  = message_filters.Subscriber('/octomap_binary',                       Octomap)

        sync = message_filters.ApproximateTimeSynchronizer([self.pcl2_sub,self.oct_sub],queue_size=5,slop=0.1)
        sync.registerCallback(self.callback_sync)
       
        ## Initialize Publishers
        self.oct_pub  = rospy.Publisher('/new_oct_binary', OctomapWithPose, queue_size=10)

        ## Initialize OcTree function with a resolution of 0.05 meters
        self.resolution = 0.05
        self.octree = octomap.OcTree(self.resolution)

        self.new_octomap_msg = OctomapWithPose()
        self.new_octomap_msg.header.frame_id = "base_link"
        self.new_octomap_msg.octomap.binary = True
        # self.new_octomap_msg.header.id = "New_Octree"


    def callback_sync(self, pcl2_msg, oct_msg):
        """
        """
        pcl_cloud = PointCloud()

        ## For loop to extract pointcloud2 data into a list of x,y,z, and
        ## store it in a pointcloud message (pcl_cloud)
        for data in pc2.read_points(pcl2_msg, skip_nans=True):
            pcl_cloud.points.append(Point32(data[0],data[1],data[2]))

        ## Parse the filtered cloud's points as a np.array. This action is required
        ## to pass as an agrument in the insertPointCloud() function.
        self.pointcloud = np.empty(shape=[len(pcl_cloud.points),3])
        for i in range(len(pcl_cloud.points)):
            self.pointcloud[i] = [pcl_cloud.points[i].x,
                                  pcl_cloud.points[i].y,
                                  pcl_cloud.points[i].z]


        ## Insert a 3D scaninto the the tree
        self.octree.insertPointCloud(pointcloud = self.pointcloud, origin = np.array([0, 0, 0], dtype=float))
        # print(type(self.octree))
        test_file = self.octree.writeBinary()


        '''
        This attribute doesn't exist in the python API.
        '''
        octomap_data = octomap.OcTreeToMsg(test_file)


        #
        self.new_octomap_msg.octomap.data = octomap_data
       

if __name__=="__main__":
    ## Initialize accumulation_map node
    rospy.init_node('accumulation_map',anonymous=True)

    ## Instantiate the AccumulationMap class
    AccumulationMap()
    rospy.spin()
