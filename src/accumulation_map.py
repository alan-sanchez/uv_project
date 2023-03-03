#!/usr/bin/env python3

# Import modules
import rospy
import numpy as np
import math
import octomap
import message_filters
import operator as op

# Import message types and other python libraries
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from best_fit import fit


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
        self.oct_center_pcl2_sub = rospy.Subscriber('/octomap_centers_pcl', PointCloud,  self.callback_oct_center_pcl)
        self.start_sub           = rospy.Subscriber('/command',             String,      self.callback_command)

        # Initialize subscribers
        self.transformed_pcl_sub = message_filters.Subscriber('/gripper_reference_pcl',  PointCloud)
        self.combined_pcl_sub    = message_filters.Subscriber('/baselink_reference_pcl', PointCloud)

        sync = message_filters.ApproximateTimeSynchronizer([self.transformed_pcl_sub,
                                                            self.combined_pcl_sub],
                                                            queue_size=5,
                                                            slop=0.1)
        sync.registerCallback(self.callback_sync)
        
        ## Initialize self.cloud for data storage in pointcloud_data callback function
        self.oct_center_pcl2 = None

        ## Initialize OcTree function with a resolution of 0.05 meters
        self.resolution = 0.05
        self.octree = octomap.OcTree(self.resolution)

        ## vector
        self.grip_vec = [0,0,-.3]
        self.mag_grip_vec = np.linalg.norm(self.grip_vec)

        ## outerbound of conical angle
        self.bound = 0.17 

        ## 
        self.command = None

        ## Bring in the UV light source model as an equation
        self.eqn_model = fit(16, plotter = False) 

        ## Set a previous time
        self.prev_time = rospy.get_time()

        ## 
        self.cell_locs = []
        self.cell_dose = []

        ## Initialize header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        ## Initialize Marker
        self.marker = Marker()
        self.marker.header = self.header
        self.marker.type = Marker.CUBE_LIST
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.02
        self.marker.scale.y = 0.02
        self.marker.scale.z = 0.02

        ## Initialize MarkerArray

    def callback_command(self, str_msg):
        """
        Function that stores the filtered point cloud and create new octree for
        castRay calculations.
        :param self: The self reference.
        :param msg: The PointCloud message type.
        """
        if str_msg.data == "start":
            ##
            self.octree.clear()
            
            ##
            rospy.sleep(0.2)
            
            ## Parse the filtered cloud's points as a np.array. This is required
            ## to pass as an agrument in the `insertPointCloud()` method.
            self.pointcloud = np.empty(shape=[len(self.oct_center_pcl.points),3])
            self.acc_map    = np.empty(shape=[len(self.oct_center_pcl.points),4])
            
            for i in range(len(self.oct_center_pcl.points)):
                self.pointcloud[i] = [self.oct_center_pcl.points[i].x,
                                      self.oct_center_pcl.points[i].y,
                                      self.oct_center_pcl.points[i].z]

            ## Insert a 3D scaninto the the tree
            self.octree.insertPointCloud(pointcloud = self.pointcloud, origin = np.array([0, 0, 0], dtype=float))
            # self.octree.writeBinary(b"before.bt")
            ##
            self.command = str_msg.data

        ##
        elif str_msg.data == "stop": 
            self.command = str_msg.data
            print(self.cell_locs, self.cell_dose)


    def callback_oct_center_pcl(self,pcl_msg):
        """
        A callback function that 
        :param self: The self reference.
        :param pcl_msg: A PointCloud message.
        """
        self.oct_center_pcl = pcl_msg
    

    def callback_sync(self, gripper_pcl, baselink_pcl):
        """
        Callback function that
        :param self: The self reference.
        :param gripper_pcl: A PointCloud message referencing the gripper_link.
        :param baselink_pcl: A PointCloud message referencing the base_link.
        """
        ## This resets the prev_time variable to None after a trajectory execution
        ## is complete. This is because there is a 2 second wait time before the user
        ## can command another trajectory
        if (rospy.get_time() - self.prev_time) > 2.0:
            self.prev_time = None

        ## A conditional statement for the first or reset of the `self.prev_time` variable.
        ## If true, then the `self.prev_time` is defined and the self.hit_list and
        ## self.accumulation_map lists are cleared out
        if self.prev_time == None:
            self.prev_time = rospy.get_time()

        ## Compute the UV time exposure
        time_exposure = abs(rospy.get_time() - self.prev_time)

        ## Create temporary empty lists and execute for loop to temporary uv dose map
        counts = []
        temp_cell_locs = []
        temp_cell_dose = []

        for tip, coord in zip(gripper_pcl.points, baselink_pcl.points):
            ## Calculate the angle (radians) between the z-axis vector and 
            ## gripper point coordinates, `tip`
            ray_length = np.linalg.norm([tip.x,tip.y,tip.z])        
            numerator = np.dot(self.grip_vec, [tip.x,tip.y,tip.z])
            denominator = self.mag_grip_vec * ray_length
            rad = np.arccos(numerator/denominator)
            
            if rad < self.bound:
                chk, key = self.octree.coordToKeyChecked(np.array([coord.x, coord.y, coord.z]))

                if chk:
                    ## computethe UV dose for the computed `rad` value
                    radius = 0.3 * math.tan(rad)
                    ir = self.eqn_model(radius)
                    dist_ratio = (0.3**2)/(ray_length**2)
                    dose = dist_ratio * time_exposure * ir

                    ## Pull cooridnates of cell key
                    pos = list(self.octree.keyToCoord(key))

                    ## Append first set of data for an empty list
                    if len(temp_cell_locs) == 0:
                        temp_cell_locs.append(pos)
                        temp_cell_dose.append(dose)
                        counts.append(1) 

                    ## Store tempoorary map   
                    else:
                        for e in temp_cell_locs:
                           
                            if pos == e:
                                ii = temp_cell_locs.index(pos)
                                temp_cell_dose[ii] += dose
                                counts[ii] += 1
                                break

                            else:
                                temp_cell_locs.append(pos)
                                temp_cell_dose.append(dose)
                                counts.append(1)
                                break
        
        ## For loop to store temporary values to actual map
        for loc, uv_dose, count in zip(temp_cell_locs,temp_cell_dose,counts):
            if loc in self.cell_locs:
                jj = self.cell_locs.index(loc)
                self.cell_dose[jj] += uv_dose/count
            else:
                self.cell_locs.append(loc)
                self.cell_dose.append(uv_dose/count)

        ## Create marker array of cells

        


        ## Set new prev_time as current time before the beginning of next loop iteration
        self.prev_time = rospy.get_time()
   
if __name__=="__main__":
    ## Initialize irradiance_vectors node
    rospy.init_node('accumulation_map',anonymous=True)

    ## Instantiate the IrradianceVectors class
    AccumulationMap()
    rospy.spin()




# print(pos)
                    # print(op.countOf(temp_hits,pos))
                    # if op.countOf(temp_hits, pos) != 0:


     #     self.octree.updateNode(value=key, update=-0.2, lazy_eval=False)
                    # print(chk)
                    # print(res)

                    # da_node = self.octree.search(np.array([loc.x, loc.y, loc.z]))
                                # print("conical angle in radians: " + str(rad))
