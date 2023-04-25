#!/usr/bin/env python3

## Import modules
import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
import octomap
import numpy as np
import math

## Import message types and other python libraries
from sensor_msgs.msg import PointCloud2, PointCloud
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point32
from std_msgs.msg import String, Header, String, ColorRGBA
from best_fit import fit
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
        self.combined_pcl2_sub   = rospy.Subscriber('/filtered_pcl2',               PointCloud2, self.callback_combined_pcl2,   queue_size=10)
        self.oct_center_pcl2_sub = rospy.Subscriber('/octomap_point_cloud_centers', PointCloud2, self.callback_oct_center_pcl2, queue_size=10)
        self.start_sub           = rospy.Subscriber('/command',                     String,      self.callback_command)
        
        ## Initialize Publisher
        self.MarkerArray_publisher = rospy.Publisher('/accumulation_map', MarkerArray, queue_size=10)
        
        ## Initialize transform listener
        self.listener = tf.TransformListener()

        ## Initialize `self.oct_center_pcl2` as None. The data from the center cells
        ## of the octomap will be stored here
        self.oct_center_pcl2 = None

        ## Initialize self.command
        self.command = "stop"

        ## Initialize OcTree function with a resolution of 0.01 meters
        self.resolution = rospy.get_param('resolution')#0.01
        self.octree = octomap.OcTree(self.resolution)

        self.measuring_ratio = (0.01/self.resolution)**2 # 0.01 is the side length of UV sensor

        ## vector the points straight down in the z direction from `uv_light_link` tf
        self.grip_vec = [0,0,-.3]
        self.mag_grip_vec = np.linalg.norm(self.grip_vec)

        ## outerbound of conical angle
        self.bound = 0.17 

        ## The required UV Dose for a UV rate constant of 0.0867 m^2/J
        ## at 99.9% disinfection rate is, 151.68 (J/m^2)
        self.required_dose = 151.66 #53.10 #151.68

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
        self.region = [[0.72, 0.55], [0.74, 0.55], [0.74, -0.55], [0.72, -0.55]]   # Sensor Array
        #self.region = [[0.70, 0.07], [0.90, 0.07], [0.90, -.113], [0.70, -.113]]  # Cone
        #self.region = [[0.75, 0.05], [0.90, 0.05], [0.90, -0.09], [0.75, -0.09]]  # Mug
        self.line = geometry.LineString(self.region)
        self.polygon = geometry.Polygon(self.line)

        self.item = True


    def callback_oct_center_pcl2(self, pcl2_msg):
        """
        A function that stores the PointCloud2 message. 
        :param self: The self reference.
        :param pcl2_msg: The PointCloud2 message.
        """
        self.oct_center_pcl2 = pcl2_msg


    def callback_command(self,str_msg):
        """
        A function that transforms a PointCloud to reference the `base_link`. 
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
            base_link_pcl = self.transform_pointcloud(pcl_cloud, "/base_link")

            ## Parse the filtered cloud's points as a np.array. This is required
            ## to pass as an agrument in the `insertPointCloud()` method.
            arr = np.empty(shape=[len(base_link_pcl),3])
            
            for i in range(len(self.oct_center_pcl.points)):
                arr[i] = [base_link_pcl[i].x,
                                      base_link_pcl[i].y,
                                      base_link_pcl[i].z]

            ## Insert a 3D scan into the the tree
            self.octree.insertPointCloud(pointcloud = arr, origin = np.array([0, 0, 0], dtype=float)) # self.octree.writeBinary(b"before.bt")

            ## Set command
            self.command = str_msg.data      

        elif str_msg.data == "stop": 
            rospy.sleep(20.0)
            ## Set command
            self.command = str_msg.data
           
            ## open file for writing, "w" is writing
            w = csv.writer(open("output.csv", "w"))

            ## loop over dictionary keys and values
            for key, val in self.acc_map_dict.items():
                ## write every key and value to file
                w.writerow([key[0],key[1],key[2],val])
                # print([key[0],key[1],key[2],val])


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
            baselink_pcl = self.transform_pointcloud(pcl_cloud, "/base_link")

            ## Transform the pointcloud message to reference the `uv_light_link`
            uv_light_pcl = self.transform_pointcloud(pcl_cloud, "/uv_light_link")

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
                # print(ray_length)
            
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

                        if self.item == True:
                            if pos[2] < 0.75:
                                continue
                        
                        if pos in temp_dict:
                            temp_dict[pos].append(dose)     
                        else:
                            temp_dict[pos] = [dose,]

            ## Create marker array of cells
            for pose_key in temp_dict:
                # dose_2 = sum(temp_dict[pose_key])*self.measuring_ratio
                # dose_2 = sum(temp_dict[pose_key])/len(temp_dict[pose_key])
                dose_value = max(temp_dict[pose_key])
                # hits = len(temp_dict[pose_key])

                # print(dose_value, dose_2, hits)
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
                    self.marker.color = ColorRGBA(1,   0,  0, 0.5)
                elif self.acc_map_dict[pose_key] < 0.5 * self.required_dose:
                    self.marker.color = ColorRGBA(1, 0.5,  0, 0.65)
                elif self.acc_map_dict[pose_key] < 0.75 * self.required_dose:
                    self.marker.color = ColorRGBA(1, 1,  0, 0.85)
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