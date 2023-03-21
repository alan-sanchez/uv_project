#!/usr/bin/env python

# Import what we need
import rospy
import tf
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point 
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class VisualizeVectors:
    def __init__(self):
        ## Initialize Subscribers
        # self.vector_sub      = rospy.Subscriber('/vectors', numpy_msg(Floats), self.visualizer)

        # Initialize Publishers
        self.waypoints_marker_pub = rospy.Publisher('waypoints_marker', Marker, queue_size=1)

        ## Initialize transform listener
        self.listener = tf.TransformListener()

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Initialize waypoint_markers and all of the other feature values
        self.waypoints_marker = Marker()
        self.waypoints_marker.header = self.header
        self.waypoints_marker.type = Marker.ARROW
        self.waypoints_marker.scale.x = 0.03
        self.waypoints_marker.scale.y = 0.01
        self.waypoints_marker.scale.z = 0.005
        self.waypoints_marker.color.a = 1
        self.waypoints_marker.color.r = 1
        self.waypoints_marker.color.g = 0
        self.waypoints_marker.color.b = 0

        ## Generate 3 circle layers of points that will serve as the LEADING points of
        ## the UV castRays. there will be 1 point in the center, 6 in the middle layer
        ## and 12 in the outer layer. These values are defined in a list named "n".
        ## "r" reresents the layer distance from the center point in centimeters
        self.n = [1, 6, 12]
        self.r = [0.0, 2.5, 5.0]

        ## Notify user that node is up and running
        rospy.loginfo("visualize_vectors node is running. Check rviz to view markers")

    def publish_marker(self):
        # Delete previous ARROW markers and publish the empty data structure
        self.waypoints_marker.action = Marker.DELETEALL
        self.waypoints_marker_pub.publish(self.waypoints_marker)

        # Set marker action to add for new ARROW markers
        self.waypoints_marker.action = Marker.ADD

        ## The `self.find_ee_pose()` function gets the translational and rotational 
        ## difference from the ee_link to the base_link. Essentially, the ee_link's
        ## coordinates in reference to the base_link tf.
        ee_trans, ee_rot = self.find_ee_pose()

        ## Extract the end effector location data.
        ## NOTE: These coordinates are referencing the base_link transform frame
        tail = Point(ee_trans[0], ee_trans[1], ee_trans[2])

        
        ## From the previous defined list, n and r, we generate the points in the
        ## self.circle_points function
        dir_vectors = self.circle_points(self.r,self.n)

        ## iteration
        i = 0
        ## hits a occupancy cube in the octree.
        for coord in dir_vectors:
            tip = Point(coord[0] + tail.x,
                        coord[1] + tail.y,
                        coord[2] + tail.z)

            ## Create new marker id and pose to be published
            self.waypoints_marker.id = i
            self.waypoints_marker.points = [tail, tip]
            self.waypoints_marker_pub.publish(self.waypoints_marker)

            ## Increment i by 1
            i+=1

            ## slow down publishing rate
            rospy.sleep(.005)

    def find_ee_pose(self):
        """
        Function that finds the pose of the ee_link relative to the base_link frame
        :param self: The self reference.

        :return [trans, rot]: The Pose message type.
        """
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform( '/base_link', '/gripper_link',rospy.Time(0))
                return [trans,rot]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
   
    def circle_points(self, r, n):
        """
        Function that generates the head positions of the CastRays.
        :param self: The self reference.
        :param r: Float value.
        :param n: Float value.

        :return concatenate: Python list.
        """
        circles = []
        for r, n in zip(r, n):
            t = np.linspace(0, 2*np.pi, n, endpoint=False)
            x = 10**-2*r * np.cos(t)    # Convert to meters by using 10**-2
            y = 10**-2*r * np.sin(t)    # Convert to meters by using 10**-2
            z = [-0.3]*len(x)
            circles.append(np.c_[x, y, z])
            concatenate = np.concatenate( circles, axis=0 )
        return concatenate    

    # def visualizer(self,msg):
    #     rospy.sleep(0.01)
    #     # print("made it here")
    #     # Delete previous ARROW markers and publish the empty data structure
    #     self.waypoints_marker.action = Marker.DELETEALL
    #     self.waypoints_marker_pub.publish(self.waypoints_marker)

    #     # Set marker action to add for new ARROW markers
    #     self.waypoints_marker.action = Marker.ADD

    #     ## Extract the end effector location data.
    #     ## NOTE: These coordinates are referencing the base_link transform frame
    #     tail = Point(msg.data[0], msg.data[1], msg.data[2])

    #     ## Extract the direction vectors and their irradiance values
    #     dir_ir_vectors = msg.data[3:]

    #     ## hits a occupancy cube in the octree.
    #     for i in range(int(len(dir_ir_vectors)/4)):
    #         ## Extract directional vector
    #         tip = Point(tail.x + dir_ir_vectors[i*4 + 0],
    #                     tail.y + dir_ir_vectors[i*4 + 1],
    #                     tail.z + dir_ir_vectors[i*4 + 2])
           
    #         # Create new marker id and pose to be published
    #         self.waypoints_marker.id = i
    #         self.waypoints_marker.points = [tail, tip]
    #         self.waypoints_marker_pub.publish(self.waypoints_marker)

if __name__=="__main__":
    # Initialize the node
    rospy.init_node('visualize_vectors')
    node = VisualizeVectors()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        node.publish_marker()
        rate.sleep()
