#!/usr/bin/env python3

# Import what we need
import rospy
import random
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose,PoseArray,Quaternion,Point
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from grid_based_sweep_coverage_path_planner import planning_animation,planning

class Waypoint_generator:
    def __init__(self):
        # Initialize Publishers
        self.waypoints_pub        = rospy.Publisher('waypoints',        PoseArray, queue_size=1)
        self.waypoints_marker_pub = rospy.Publisher('waypoints_marker', Marker,    queue_size=1)

        # Setup header
        self.header = Header()
        self.header.frame_id = "/base_link"
        self.header.stamp = rospy.Time.now()

        # Intialize the X and Y coordinates of the 2D data points
        self.X = [.75, 1, 1, .75, .75 ]
        self.Y = [-.2,  -.2, .2, .2, -.2]

        # Set the waypoint resolution (distance between points)
        self.resolution = .045

        # Initialize waypoints as a PoseArray
        self.waypoints = PoseArray()
        self.waypoints.header = self.header

        # Initialize waypoint_markers and all of the other feature values
        self.waypoints_marker = Marker()
        self.waypoints_marker.header = self.header
        self.waypoints_marker.type = Marker.LINE_STRIP
        self.waypoints_marker.action = Marker.ADD
        self.waypoints_marker.id = 1
        self.waypoints_marker.scale.x = .01
        self.waypoints_marker.scale.y = .01
        self.waypoints_marker.color.a = 1
        self.waypoints_marker.color.r = 0
        self.waypoints_marker.color.g = 0
        self.waypoints_marker.color.b = 1.0
   
    def waypoint_planner(self):
        # Acquire the planned x an y values from the planning function
        px,py = planning(self.X, self.Y, self.resolution)
        # planning_animation(self.X,self.Y, self.resolution)

        # Create lists and array of waypoints to publish
        poses = []
        marker_list = []

        # Begin dimension increase for 2D coordinates (px, py)
        for i in range(len(px)):
            # poses append to poses (Pose Array)
            p = Pose()
            p.position.x = round(px[i],2)
            p.position.y = round(-1*py[i],2)
            p.position.z = 1.12
            p.orientation.x = 0.0
            p.orientation.y = 0.0
            p.orientation.z = 0.0
            p.orientation.w = 1.0
            poses.append(p)

            # Append position values for the marker
            marker_list.append(Point(p.position.x, p.position.y, p.position.z))
        print(len(poses))
        # assisn poses to the PoseArray, self,waypoints.
        self.waypoints.poses = poses

        # Publish poses for computeCartesianPath
        self.waypoints_pub.publish(self.waypoints)

        # Publish markers for waypoints
        self.waypoints_marker.points = marker_list
        self.waypoints_marker_pub.publish(self.waypoints_marker)


if __name__=="__main__":
    # Initialize the node
    rospy.init_node('waypoint_generator')

    # Declare object from Region class
    region = Waypoint_generator()

    # Set rospy rate
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Wait for user input before generating new disinfection region
        print("Press enter to send waypoints")
        input()

        # Run the convex hull function
        region.waypoint_planner()
        rate.sleep()
