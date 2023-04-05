#!/usr/bin/env python

# Import modules
import rospy
import numpy as np
import tf
import math
import sensor_msgs.point_cloud2 as pc2

# Import message types and other python libraries
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Point32

class IrradianceVectors(object):
    """
    A class that publishes the direction and irridance values of the UV vectors
    whose origins are from the end effector.
    """
    def __init__(self):
        """
        A function that initialises  the subscriber, publisher, and other variables.
        :param self: The self reference.
        """
        ## Initialize Subscriber
        self.combined_pcl2_sub = rospy.Subscriber('/combined_filtered_image_and_depthmap', PointCloud2, self.callback_pcl2)
        
        ## Instantiate a `tf.TransformListener` object
        self.listener = tf.TransformListener()

        ## Initialize self.cloud for data storage in pointcloud_data callback function
        self.pcl2_cloud = None

        ## Create a list that represents a vector in the -z direction
        self.a = [0,0,-.3]
        self.mag_a = np.linalg.norm(self.a)

        ## outerbound of conical angle
        self.bound = 0.17 

    def callback_pcl2(self, pcl2_msg):
        """
        Callback function that stores the pointcloud2 message.
        :param self: The self reference.
        :param pcl2_msg: The PointCloud2 message type.
        """
        ## Store PointCloud2 message type
        self.pcl2_cloud = pcl2_msg

        ## Initialize a new point cloud message type to store position data.
        pcl_cloud = PointCloud()
        pcl_cloud.header = self.pcl2_cloud.header

        ## For loop to extract pointcloud2 data into a list of x,y,z, and
        ## store it in a pointcloud message (pcl_cloud)
        for data in pc2.read_points(self.pcl2_cloud, skip_nans=True):
            pcl_cloud.points.append(Point32(data[0],data[1],data[2]))

        ## Transform the pointcloud message to reference the base_link
        transformed_cloud = self.transform_pointcloud(pcl_cloud)

        ## Compute conical angle
        for loc in transformed_cloud.points:
            numerator = np.dot(self.a, [loc.x,loc.y,loc.z])
            denominator = self.mag_a * np.linalg.norm([loc.x,loc.y,loc.z])
            rad = np.arccos(numerator/denominator)

            print("conical angle in radians: " + str(rad))

    def transform_pointcloud(self,pcl_cloud):
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
                ## to the base_link
                new_cloud = self.listener.transformPointCloud("/gripper_link" ,pcl_cloud)
                return new_cloud
            except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
                pass    


if __name__=="__main__":
    ## Initialize irradiance_vectors node
    rospy.init_node('irradiance_vectors',anonymous=True)

    ## Instantiate the IrradianceVectors class
    IrradianceVectors()
    rospy.spin()