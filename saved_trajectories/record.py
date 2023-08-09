#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import pickle

from itertools import compress


class ArmRecorder:
	def __init__(self):
		## Initialize in a stopped state.
		self.recording = False

		## Create space to store things.
		self.reset()

		## Subscribe to the joint states.
		self.sub = rospy.Subscriber('joint_states', JointState, self.joint_callback, queue_size=1)

		## Initialize Publisher
        self.command_pub = rospy.Publisher('/command', String, queue_size=1)

	def reset(self):
		self.trajectory = []

	def start(self):
		self.recording = True
		rospy.loginfo('{}: Starting recording.'.format(self.__class__.__name__))

	def stop(self):
		self.recording = False
		rospy.loginfo('{}: Stopping recording.'.format(self.__class__.__name__))

	def joint_callback(self, msg):
		# Just save the whole message.
		if len(msg.name) == 13: # Needed a conditional statement because the gripper joints are published faster than the rest of the joints. 
			self.trajectory.append(msg)

	def save(self, filename):
		ARM_JOINTS = (
			'torso_lift_joint',
			'shoulder_pan_joint',
			'shoulder_lift_joint',
			'upperarm_roll_joint',
			'elbow_flex_joint',
			'forearm_roll_joint',
			'wrist_flex_joint',
			'wrist_roll_joint'
		)

		# Make a mask for the arm joint elements.
		mask = [name in ARM_JOINTS for name in self.trajectory[0].name]

		# Create a trajectory.
		trajectory = JointTrajectory()
		trajectory.header = self.trajectory[0].header
		trajectory.joint_names = list(compress(self.trajectory[0].name, mask))

		# Build the collection of trajectory points.
		start_time = self.trajectory[0].header.stamp
		for point in self.trajectory:
			trajectory_point = JointTrajectoryPoint()
			trajectory_point.positions = list(compress(point.position, mask))

			trajectory_point.velocities = list(compress(point.velocity, mask))
			trajectory_point.time_from_start = point.header.stamp - start_time
			trajectory.points.append(trajectory_point)
		
		# Save it out to the file.  We're going to pickle for now.
		with open(filename, 'wb') as f:
			pickle.dump(trajectory, f)

		rospy.loginfo('{}: Saved {} waypoints to {}.'.format(self.__class__.__name__, len(trajectory.points), filename))


if __name__ == '__main__':
	rospy.init_node('record')

	recorder = ArmRecorder()
	recorder.start()

	rospy.spin()

	recorder.stop()

	# Get user input for the filename
    filename = input("Enter a filename to save: ")
	recorder.save(filename + '.pkl')
