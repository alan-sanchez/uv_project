#!/usr/bin/env python3

# Import what we need
import rospy
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray,Quaternion,Point, PolygonStamped
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

class VisualizeVectors:
    def __init__(self):
        # Initialize Subscribers
        self.vector_sub      = rospy.Subscriber('/vectors', numpy_msg(Floats), self.visualizer)

        # Initialize Publishers
        self.waypoints_marker_pub = rospy.Publisher('waypoints_marker', Marker    , queue_size=1)

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

    def visualizer(self):
        # Delete previous ARROW markers and publish the empty data structure
        self.waypoints_marker.action = Marker.DELETEALL
        self.waypoints_marker_pub.publish(self.waypoints_marker)

        # Set marker action to add for new ARROW markers
        self.waypoints_marker.action = Marker.ADD

        # Create lists for waypoints and waypoint markers that will be publish
        poses = []
        marker_list = []

        ## Extract the end effector location data.
        ## NOTE: These coordinates are referencing the base_link transform frame
        tail = Point(msg.data[0], msg.data[1], msg.data[2])

        ## Extract the direction vectors and their irradiance values
        dir_ir_vectors = msg.data[3:]

        ## hits a occupancy cube in the octree.
        for i in range(int(len(dir_ir_vectors)/4)):
            ## Extract directional vector
            tip = Point(dir_ir_vectors[j*4 + 0],
                        dir_ir_vectors[j*4 + 1],
                        dir_ir_vectors[j*4 + 2])
            # Include characteristics of a pose
            p = Pose()
            p.position.x = self.cloud_x[index] + offset[0]
            p.position.y = self.cloud_y[index] + offset[1]
            p.position.z = self.cloud_z[index] + offset[2]
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            poses.append(p)

            # Create new marker id and pose to be published
            self.waypoints_marker.id = i
            self.waypoints_marker.points = [tail, ]
            self.waypoints_marker_pub.publish(self.waypoints_marker)


            # assisn poses to the PoseArray, self,waypoints.
            self.waypoints.poses = poses

            # Publish poses for computeCartesianPath
            self.waypoints_pub.publish(self.waypoints)

            # Clear out cloud data for new updated data
            del self.cloud_x[:], self.cloud_y[:], self.cloud_z[:]



if __name__=="__main__":
    # Initialize the node
    rospy.init_node('nonplanar_waypoint_generator')
    Nonplanar_waypoint_generator()
    rospy.spin()
