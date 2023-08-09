#!/usr/bin/env python


import rospy
import actionlib
from std_msgs.msg import String

import moveit_commander

from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, JointTolerance

## Import message types and other python librarires
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from std_msgs.msg import String

import pickle


class ArmReplayer:
	def __init__(self):
		## Initialize Publisher
		self.command_pub = rospy.Publisher('/command', String, queue_size=10)
		
		##
		self.action_client = actionlib.SimpleActionClient('arm_with_torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		self.action_client.wait_for_server()
		rospy.loginfo('{}: Action client ready.'.format(self.__class__.__name__))

		## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
		## to one group of joints.
		self.group = moveit_commander.MoveGroupCommander("arm_with_torso")
		self.group.set_end_effector_link("gripper_link")

	def replay(self, filename, scale_factor=1):
		# Load trajectory from the file.
		with open(filename, 'rb') as f:
			trajectory = pickle.load(f)

		trajectory.points = trajectory.points[::100]
		trajectory.header.seq = 0
		trajectory.header.stamp.secs = 0
		trajectory.header.stamp.nsecs = 0 
		trajectory.header.frame_id="base_link"
		rospy.loginfo('{}: Loaded trajectory with {} waypoints.'.format(self.__class__.__name__, len(trajectory.points)))

		#Scale the durations of the actions
		for i in range(len(trajectory.points)):
			trajectory.points[i].time_from_start *= scale_factor

		# Create a goal.
		goal = FollowJointTrajectoryGoal()
		goal.trajectory = trajectory

		# Send the goal to the action client, and wait for it to finish.
		self.command_pub.publish("start")
		self.action_client.send_goal(goal)
		self.command_pub.publish("stop")
		self.action_client.wait_for_result()
		result = self.action_client.get_result()

		if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
			rospy.loginfo('{}: Action call successful.'.format(self.__class__.__name__))
		else:
			rospy.loginfo('{}: Action call failed with {} ({}).'.format(self.__class__.__name__, result.error_code, result.error_string))

	def init_pose(self):
		"""
		Function that sends a joint goal that moves the Fetch's arm and torso to
		the initial position.
		:param self: The self reference.
		:param vel: Float value for arm velocity.
		"""
		joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
				  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

		pose =[.15, 1.41, 0.30, -0.22, -2.25, -1.56, 1.80, -0.37,]

		self.group.set_max_velocity_scaling_factor(.2)
		self.group.set_joint_value_target(pose)
		plan = self.group.plan()      
		self.group.execute(plan)


if __name__ == '__main__':
	rospy.init_node('replay')

	replayer = ArmReplayer()

	while True:
		## 
		print("")
		print("====== Press 'Enter' to return to initial position =======")
		raw_input()
		motion.init_pose()

		## Get user input for the filename
		filename = raw_input("Enter the filename to replay (or 'exit' to quit): ")
		if filename.lower() == 'exit':
			break

		replayer.replay(filename + '.pkl')