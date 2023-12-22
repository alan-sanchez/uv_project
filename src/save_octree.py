#!/usr/bin/env python3

## Import modules
import rospy
import sensor_msgs.point_cloud2 as pc2
import octomap
import tf
import numpy as np

## Import message types and other python libraries
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Point32
from shapely import geometry


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
        self.oct_center_pcl2_sub = rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.callback_oct_center_pcl2, queue_size=10)

        ## Initialize transform listener
        self.listener = tf.TransformListener()

        ## Initialize OcTree function with a resolution of 0.01 meters
        self.resolution = 0.01 #rospy.get_param('resolution')
        self.octree = octomap.OcTree(self.resolution)

        ## Create sensor array region 
        #self.region = [[0.72, 0.55], [0.74, 0.55], [0.74, -0.55], [0.72, -0.55]]  # Sensor Array
        #self.region = [[0.70, 0.07], [0.90, 0.07], [0.90, -.113], [0.70, -.113]]  # Cone
        #self.region = [[0.75, 0.05], [0.90, 0.05], [0.90, -0.09], [0.75, -0.09]]  # Mug
        self.region = [[0.75, 0.20], [1.00, 0.20], [1.00, -0.20], [0.75, -0.20]]  # Table
        self.lower_z = 0.74
        self.upper_z = 0.75

        self.line = geometry.LineString(self.region)
        self.polygon = geometry.Polygon(self.line)
      


    def callback_oct_center_pcl2(self, pcl2_msg):
        """
        A function that stores the PointCloud2 message. 
        :param self: The self reference.
        :param pcl2_msg: The PointCloud2 message.
        """
        ## Initialize a new point cloud message type to store position data.
        pcl_cloud = PointCloud()
        pcl_cloud.header = pcl2_msg.header
        
        ## For loop to extract pointcloud2 data into a list of x,y,z, and
        ## store it in a pointcloud message (pcl_cloud)
        for data in pc2.read_points(pcl2_msg, skip_nans=True):
            pcl_cloud.points.append(Point32(data[0],data[1],data[2]))
            
        ## Transform the pointcloud message to reference the `base_link`
        base_link_center_pcl = self.transform_pointcloud(pcl_cloud, "/base_link")

        temp_list = []
        # print(type(base_link_center_pcl))
        for base_coord in base_link_center_pcl.points:
            point = geometry.Point(base_coord.x,base_coord.y)

            if self.polygon.contains(point) == False:
                continue
            if self.lower_z > base_coord.z or base_coord.z > self.upper_z:
                # print("made it here")
                continue
            
            temp_list.append([base_coord.x, base_coord.y, base_coord.z])
            if base_coord.z < .6 :
                print(base_coord.z)
          
        ## Parse the filtered cloud's points as a np.array. This is required
        ## to pass as an agrument in the `insertPointCloud()` method.
        arr = np.empty(shape=[len(temp_list),3])

        for i, ele in enumerate(temp_list):
            arr[i] = [ele[0],ele[1],ele[2]]
        print(len(arr))
        print(len(base_link_center_pcl.points))
        ## Insert a 3D scan into the the tree
        self.octree.insertPointCloud(pointcloud = arr, origin = np.array([0, 0, 0], dtype=float)) # 
        self.octree.writeBinary(b"oct.bt")
        rospy.loginfo("Octree saved")
        rospy.signal_shutdown("Your reason for shutting down")



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
    rospy.init_node('save_octree',anonymous=True)

    ## Instantiate the IrradianceVectors class
    TransformPCL()
    rospy.spin()