#!/usr/bin/env python


import rospy
import actionlib

from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, JointTolerance

import pickle


class ArmReplayer:
	def __init__(self):
		self.action_client = actionlib.SimpleActionClient('arm_with_torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		self.action_client.wait_for_server()
		rospy.loginfo('{}: Action client ready.'.format(self.__class__.__name__))

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

		# You might have to also set path_tolerance, goal_tolerance and goal_time_tolerance here
		# See http://docs.ros.org/en/api/control_msgs/html/action/FollowJointTrajectory.html

		# Send the goal to the action client, and wait for it to finish.
		self.action_client.send_goal(goal)
		self.action_client.wait_for_result()
		result = self.action_client.get_result()

		if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
			rospy.loginfo('{}: Action call successful.'.format(self.__class__.__name__))
		else:
			rospy.loginfo('{}: Action call failed with {} ({}).'.format(self.__class__.__name__, result.error_code, result.error_string))


if __name__ == '__main__':
	rospy.init_node('replay')

	replayer = ArmReplayer()

	# rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     ## Pause for 2 seconds after the motion is completed
    #     rospy.sleep(2)

    #     print("")
    #     print("====== Press 'Enter' to return to initial position =======")
    #     raw_input()
    #     motion.init_pose()

    #     ## 
    #     print("\nType the number of the trajectory to play back.\n1: Table\n2: Cone\n3: Mug\n4: Sensor Array\n5: Cone User Input\n6: Mug Uwer Input\n\n")
    #     choice = input("Selection: ")
    #     if choice == 1 or choice == 2 or choice == 3 or choice == 4 or choice == 5 or choice == 6:
    #         motion.traj_playback(choice)
    #     else:
    #         print("Enter 1, 2, 3, or 4")
        
    #     rate.sleep()

	replayer.replay('test.pkl')