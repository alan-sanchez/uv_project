#!/usr/bin/env python

## Import modules
import rospy
import tf
import sensor_msgs.point_cloud2 as pc2

## Import message types and other python libraries
from sensor_msgs.msg import PointCloud2, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
from std_msgs.msg import String


class TransformPCL(object):
    """
    A class that publishes the direction and irridance values of the UV vectors
    whose origins are from the end effector.
    """
    def __init__(self):
        """
        A function that initializes the subscriber, publisher, and other variables.
        :param self: The self reference.
        """
        ## Initialize Subscribers
        self.combined_pcl2_sub   = rospy.Subscriber('/filtered_pcl2',               PointCloud2, self.callback_combined_pcl2,   queue_size=10)
        self.oct_center_pcl2_sub = rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.callback_oct_center_pcl2, queue_size=10)
        self.start_sub           = rospy.Subscriber('/command',                     String,      self.callback_command)
        
        ## Initialize PointCloud Publishers
        self.baselink_pcl_pub = rospy.Publisher("/baselink_reference_pcl", PointCloud, queue_size=5)
        self.gripper_pcl_pub  = rospy.Publisher("/gripper_reference_pcl",  PointCloud, queue_size=5)
        self.oct_centers_pub  = rospy.Publisher("/octomap_centers_pcl",    PointCloud, queue_size=5)
        
        ## Initialize transform listener
        self.listener = tf.TransformListener()

        ## Initialize `self.oct_center_pcl2` as None. The data from the center cells
        ## of the octomap will be stored here
        self.oct_center_pcl2 = None

        ## Initialize self.command
        self.command = "stop"

    def callback_oct_center_pcl2(self, pcl2_msg):
        """
        A function that stores the PointCloud2 message types which represents 
        the center of the cells from the octomap.
        :param self: The self reference.
        :param pcl2_msg: The PointCloud2 message.
        """
        self.oct_center_pcl2 = pcl2_msg

    def callback_command(self,str_msg):
        """
        A function that publishes the octomap center cells as a PointCloud message type.
        The function also transforms the coordinates to reference the `base_link`. 
        :param self: The self reference.
        :param str_msg: A String message type.
        """
        if str_msg.data == "start":
            ## Initialize a new point cloud message type to store position data.
            pcl_cloud = PointCloud()
            pcl_cloud.header = self.oct_center_pcl2.header
            
            ## For loop to extract pointcloud2 data into a list of x,y,z, and
            ## store it in a pointcloud message (pcl_cloud)
            for data in pc2.read_points(self.oct_center_pcl2, skip_nans=True):
                pcl_cloud.points.append(Point32(data[0],data[1],data[2]))
               
            ## Transform the pointcloud message to reference the `base_link`
            octomap_center_pcl = self.transform_pointcloud(pcl_cloud, "/base_link")
            self.oct_centers_pub.publish(octomap_center_pcl)       

            self.command = str_msg.data 

        else:
            self.command = str_msg.data

    def callback_combined_pcl2(self, pcl2_msg):
        """
        Callback function that stores the PointCloud2 message of the combined
        filtered image and depth map. This function also transforms the cooridnates 
        from its original transform frame to the `base_link` and `uv_light_link`. 
        :param self: The self reference.
        :param pcl2_msg: The PointCloud2 message type.
        """
        if self.command == "start":

            ## Initialize a new point cloud message type to store position data.
            pcl_cloud = PointCloud()
            pcl_cloud.header = pcl2_msg.header  

            ## For loop to extract pointcloud2 data into a list of x,y,z, and
            ## store it in a pointcloud message (pcl_cloud)
            # count = 0
            for data in pc2.read_points(pcl2_msg, skip_nans=True):
                pcl_cloud.points.append(Point32(data[0],data[1],data[2]))

            ## Transform the pointcloud message to reference the `base_link`
            base_cloud = self.transform_pointcloud(pcl_cloud, "/base_link")
            self.baselink_pcl_pub.publish(base_cloud)

            ## Transform the pointcloud message to reference the `uv_light_link`
            transformed_cloud = self.transform_pointcloud(pcl_cloud,"/uv_light_link")
            self.gripper_pcl_pub.publish(transformed_cloud)

    def transform_pointcloud(self,pcl_cloud, target_frame):
        """
        Function that transform a PointCloud coordinates to a target transform frame
        :param self: The self reference.
        :param pcl_cloud: The PointCloud message.
        :param target_frame: A string message. 

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