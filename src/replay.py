#!/usr/bin/env python3

## Import modules
import sys
import rospy
import yaml 
import signal
import actionlib
import pickle

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, JointTolerance
from std_msgs.msg import String
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene


class ArmReplayer(object):
	def __init__(self):
		## Initialize Publisher
		self.command_pub = rospy.Publisher('/command', String, queue_size=10)
		
		##
		self.action_client = actionlib.SimpleActionClient('arm_with_torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		self.action_client.wait_for_server()
		rospy.loginfo('{}: Action client ready.'.format(self.__class__.__name__))

		rospy.loginfo("Waiting for MoveIt...")
		self.client = MoveGroupInterface("arm_with_torso", "base_link")
		rospy.loginfo("...connected")


	def replay(self, filename, scale_factor=1):
		# Load trajectory from the file.
		with open(filename, 'rb') as f:
			trajectory = pickle.load(f)

		
		# Define the position to check
		target_position = [0.15, 1.41, 0.3, -0.22, -2.25, -1.56, 1.8, -0.37]
		position_indices_to_remove = []

		tolerance = 0.01  # Tolerance value

		for i, point in enumerate(trajectory.points):
			is_within_tolerance = True
			for j, pos in enumerate(point.positions):
				if abs(pos - target_position[j]) > tolerance:
					is_within_tolerance = False
					break  # Exit inner loop once a mismatch is found
					
			if is_within_tolerance:
				position_indices_to_remove.append(i)  # Append the index to remove if position is within tolerance

		# Removing points from the trajectory
		for index in reversed(position_indices_to_remove):
			del trajectory.points[index]

		##
		trajectory.points = trajectory.points[::100] # Getting every 100 point at 100hz (equivalent to 1hz)
		trajectory.header.seq = 0
		trajectory.header.stamp.secs = 0
		trajectory.header.stamp.nsecs = 0 
		trajectory.header.frame_id="base_link"
		rospy.loginfo('{}: Loaded trajectory with {} waypoints.'.format(self.__class__.__name__, len(trajectory.points)))


		# print(trajectory.points)
		#Scale the durations of the actions
		for i in range(len(trajectory.points)):
			trajectory.points[i].time_from_start = rospy.Duration(i)
			trajectory.points[i].time_from_start *= scale_factor
			# print(trajectory.points[i].time_from_start )

		# Create a goal.
		goal = FollowJointTrajectoryGoal()
		goal.trajectory = trajectory

		# Send the goal to the action client, and wait for it to finish.
		self.command_pub.publish("start")
		self.action_client.send_goal(goal)
		self.action_client.wait_for_result()
		self.command_pub.publish("stop")
		result = self.action_client.get_result()

		if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
			rospy.loginfo('{}: Action call successful.'.format(self.__class__.__name__))
		else:
			rospy.loginfo('{}: Action call failed with {} ({}).'.format(self.__class__.__name__, result.error_code, result.error_string))

	def init_pose(self, vel = 0.2):
		"""
		Function that sends a joint goal that moves the Fetch's arm and torso to
		the initial position.
		:param self: The self reference.
		:param vel: Float value for arm velocity.
		"""
		
		## Padding does not work (especially for self collisions)
		## So we are adding a box above the base of the robot
		scene = PlanningSceneInterface("base_link")
		scene.addBox("keepout", 0.25, 0.5, 0.09, 0.15, 0.0, 0.375)

		joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
					"elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
		pose =[.15, 1.41, 0.30, -0.22, -2.25, -1.56, 1.80, -0.37,]
		while not rospy.is_shutdown():
			result = self.client.moveToJointPosition(joints,
														pose,
														0.0,
														max_velocity_scaling_factor=vel)
			if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
				scene.removeCollisionObject("keepout")
				return 0 

	
if __name__ == '__main__':
	rospy.init_node('replay')

	replayer = ArmReplayer()

	while not rospy.is_shutdown():
		print("")
		print("====== Press 'Enter' to return to initial position =======")
		input()
		replayer.init_pose()

		## Get user input for the filename
		filename = input("Enter the filename to replay (or 'exit' to quit): ")
		if filename.lower() == 'exit':
			break
		
		replayer.replay(filename + '.pkl')
