#!/usr/bin/env python

# Import modules
import rospy
import numpy as np
import tf
import sensor_msgs.point_cloud2 as pc2

# Import message types and other python libraries
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import String


class TransformPCL(object):
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
        self.combined_pcl2_sub   = rospy.Subscriber('/combined_filtered_image_and_depthmap', PointCloud2, self.callback_pcl2)
        self.oct_center_pcl2_sub = rospy.Subscriber('/octomap_point_cloud_centers',          PointCloud2, self.callback_oct_center_pcl2)
        self.start_sub           = rospy.Subscriber('/command',                              String,      self.callback_command)
        
        ## Initialize PointCloud Publisher
        self.transformed_pcl_pub = rospy.Publisher("/transformed_cloud",    PointCloud, queue_size=1)
        self.combined_pcl_pub    = rospy.Publisher("/combined_point_cloud", PointCloud, queue_size=1)
        self.oct_pcl_center_pub  = rospy.Publisher("/octomap_pcl_centers",  PointCloud, queue_size=1)
        
        ## Initialize transform listener
        self.listener = tf.TransformListener()

        ## Initialize self.cloud for data storage in pointcloud_data callback function
        self.pcl2_cloud = None

        ## 
        self.command = "stop"

    def callback_oct_center_pcl2(self, pcl2_msg):
        """
        """
        self.oct_center_pcl2 = pcl2_msg

    def callback_command(self,str_msg):
        """
        """
        if str_msg.data == "start":
            ## Initialize a new point cloud message type to store position data.
            pcl_cloud = PointCloud()
            pcl_cloud.header = self.oct_center_pcl2.header

            ## For loop to extract pointcloud2 data into a list of x,y,z, and
            ## store it in a pointcloud message (pcl_cloud)
            for data in pc2.read_points(self.oct_center_pcl2, skip_nans=True):
                pcl_cloud.points.append(Point32(data[0],data[1],data[2]))

            ## 
            octomap_center_pcl = self.transform_pointcloud(pcl_cloud, "/base_link")
            self.oct_pcl_center_pub.publish(octomap_center_pcl)       

            self.command = str_msg.data 

        else:
            self.command = str_msg.data

    def callback_pcl2(self, pcl2_msg):
        """
        Callback function that stores the pointcloud2 message.
        :param self: The self reference.
        :param pcl2_msg: The PointCloud2 message type.
        """
        ## 
        self.pcl2_cloud = pcl2_msg

        ## 
        if self.command == "start":

            ## Initialize a new point cloud message type to store position data.
            pcl_cloud = PointCloud()
            pcl_cloud.header = self.pcl2_cloud.header
            pcl_cloud.header.stamp=rospy.Time.now()

            ## For loop to extract pointcloud2 data into a list of x,y,z, and
            ## store it in a pointcloud message (pcl_cloud)
            for data in pc2.read_points(self.pcl2_cloud, skip_nans=True):
                pcl_cloud.points.append(Point32(data[0],data[1],data[2]))

            ##
            base_cloud = self.transform_pointcloud(pcl_cloud, "/base_link")
            self.combined_pcl_pub.publish(base_cloud)

            ## Transform the pointcloud message to reference the base_link
            transformed_cloud = self.transform_pointcloud(pcl_cloud,"/gripper_link")
            self.transformed_pcl_pub.publish(transformed_cloud)

    def transform_pointcloud(self,pcl_cloud, target_frame):
        """
        Function that stores the pointcloud2 message.
        :param self: The self reference.
        :param pcl_cloud: The PointCloud message.

        :returns new_cloud: PointCloud message.
        """
        pcl_cloud.header.stamp=rospy.Time.now()
        while not rospy.is_shutdown():
            try:
                ## run the transformPointCloud() function to change the referene frame
                ## to the target frame
                new_cloud = self.listener.transformPointCloud(target_frame ,pcl_cloud)
                return new_cloud
            except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
                pass    


if __name__=="__main__":
    ## Initialize transform_pcl node
    rospy.init_node('transform_pcl',anonymous=True)

    ## Instantiate the IrradianceVectors class
    TransformPCL()
    rospy.spin()