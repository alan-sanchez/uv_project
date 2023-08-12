#!/usr/bin/env python

import sys
import actionlib
import subprocess
import rospy
import moveit_commander
import moveit_msgs.msg
import numpy as np

from threading import Thread
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene, RobotTrajectory
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray

class ExecutePath(object):
    def __init__(self):
        super(ExecutePath, self).__init__()
        # Initialize subscribers
        self.waypoints_sub     = rospy.Subscriber('waypoints', PoseArray, self.waypoint_callback)

        # Publisher
        self.duration_pub  = rospy.Publisher('duration', Float32, queue_size=10)

        # First initialize `moveit_commander`
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a `RobotCommander`_ object. This object is the outer-level
        # interface to the robot
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.
        self.group = moveit_commander.MoveGroupCommander("arm_with_torso")
        self.group.set_end_effector_link("gripper_link")
        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        # Intialize the plan
        self.plan = None

        rospy.loginfo("Table path up and running")

        

    def waypoint_callback(self,msg):
        print('made it here')
        waypoints = []
        # Append poses to a list
        for i in range(len(msg.poses)):
            waypoints.append(msg.poses[i])   

        ## Cartesian Paths
        (plan, fraction) = self.group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.1,              # eef_step
                                        0.00)             # jump_threshold

        # plan = self.group.retime_trajectory(self.robot.get_current_state(),plan,.57)
        plan = self.group.retime_trajectory(self.robot.get_current_state(),
                                            plan,
                                            velocity_scaling_factor = 0.18)
    
        
        self.group.execute(plan, wait=True)


#   def execute_plan(self, plan):
#  #     print ("WAYPOINTS ARE:", len(self.waypoints), self.waypoints)
#       self.group.execute(plan, wait=True)



if __name__=="__main__":
    rospy.init_node('execute_path',anonymous=True)
    ExecutePath()
    rospy.spin()
