#!/usr/bin/env python3

## Import modules
import rospy
import tf
import sensor_msgs.point_cloud2 as pc2

## Import message types and other python libraries
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Point32


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
        self.combined_pcl2_sub   = rospy.Subscriber('/filtered_pcl2', PointCloud2, self.callback_combined_pcl2,   queue_size=1)
        # self.combined_pcl2_sub   = rospy.Subscriber('/camera/depth_registered/points',PointCloud2, self.callback_combined_pcl2,   queue_size=1)
        
        ## Initialize PointCloud Publishers
        self.baselink_pcl_pub = rospy.Publisher("/baselink_reference_pcl", PointCloud, queue_size=1)
        self.gripper_pcl_pub  = rospy.Publisher("/gripper_reference_pcl",  PointCloud, queue_size=1)
        
        ## Initialize transform listener
        self.listener = tf.TransformListener()

    def callback_combined_pcl2(self, pcl2_msg):
        """
        Callback function that stores the PointCloud2 message of the combined
        filtered image and depth map. This function also transforms the cooridnates 
        from its original transform frame to the `base_link` and `uv_light_link`. 
        :param self: The self reference.
        :param pcl2_msg: The PointCloud2 message type.
        """
        ## Initialize a new point cloud message type to store position data.
        pcl_cloud = PointCloud()
        pcl_cloud.header = pcl2_msg.header
    
        start = rospy.get_time()
        for data in pc2.read_points(pcl2_msg, skip_nans=True):
            pcl_cloud.points.append(Point32(data[0],data[1],data[2]))
        time_diff = rospy.get_time() - start  

        start = rospy.get_time()
        ## Transform the pointcloud message to reference the `base_link`
        base_cloud = self.transform_pointcloud(pcl_cloud, "/base_link")
        self.baselink_pcl_pub.publish(base_cloud)
        time_diff2 = rospy.get_time() - start   

        start = rospy.get_time()
        ## Transform the pointcloud message to reference the `uv_light_link`
        # transformed_cloud = self.transform_pointcloud(pcl_cloud,"/uv_light_link")
        self.gripper_pcl_pub.publish(base_cloud)
        time_diff3 = rospy.get_time()  - start  

        # print(time_diff, time_diff2, time_diff3)

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
    rospy.init_node('transform_pcl_viewer',anonymous=True)

    ## Instantiate the IrradianceVectors class
    TransformPCL()
    rospy.spin()