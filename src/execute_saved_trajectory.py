#!/usr/bin/env python

## Import modules
import sys
import rospy
import yaml 
import signal
import actionlib
import pickle

import moveit_commander
import moveit_msgs.msg
from copy import deepcopy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, JointTolerance


## Import message types and other python librarires
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from std_msgs.msg import String

class ExecutePath(object):
    """
    Class that commands the Fetch robot to execute a joint trajectory based on
    subscribes pose goals.
    """
    def __init__(self):
        """
        A function that initializes subscriber, publisher, moveit_commander,and
        planning scene.
        :param self: The self reference.
        """
        super(ExecutePath, self).__init__()
        # self.action_client = actionlib.SimpleActionClient('arm_with_torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # self.action_client.wait_for_server()
        # rospy.loginfo('{}: Action client ready.'.format(self.__class__.__name__))
        
        ## Initialize Publisher
        self.command_pub = rospy.Publisher('/command', String, queue_size=10)

        ## First initialize `moveit_commander`
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a `RobotCommander`_ object. This object is the outer-level
        ## interface to the robot
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

    def init_pose(self, vel = 0.2):
        """
        Function that sends a joint goal that moves the Fetch's arm and torso to
        the initial position.
        :param self: The self reference.
        :param vel: Float value for arm velocity.
        """
        # rospy.loginfo("Waiting for MoveIt...")
        # self.client = MoveGroupInterface("arm_with_torso", "base_link")
        # rospy.loginfo("...connected")

        ## Padding does not work (especially for self collisions)
        ## So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")
        scene.addBox("keepout", 0.25, 0.5, 0.09, 0.15, 0.0, 0.375)

        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose =[.15, 1.41, 0.30, -0.22, -2.25, -1.56, 1.80, -0.37,]

        # with open("test.pkl", 'rb') as f:
		# 	trajectory = pickle.load(f)

        
        # trajectory.points = trajectory.points[::50]
        self.group.set_max_velocity_scaling_factor(.2)
        self.group.set_joint_value_target(pose)
        plan = self.group.plan()
        # print(plan.joint_trajectory.header)
        # plan.joint_trajectory = trajectory
        # plan.joint_trajectory.header.seq = 0
        # plan.joint_trajectory.header.stamp.secs = 0
        # plan.joint_trajectory.header.stamp.nsecs = 0
        # plan.joint_trajectory.header.frame_id="base_link"
        # print(plan.joint_trajectory.header)
       
        # plan2 = deepcopy(plan)

        #for i in range(len(plan.joint_trajectory.points)):
        #    plan.joint_trajectory.points[i].accelerations = []
        # print(plan.joint_trajectory.points)
      
        self.group.execute(plan)

        # goal = FollowJointTrajectoryGoal()
        # goal.trajectory = plan.joint_trajectory

        # # You might have to also set path_tolerance, goal_tolerance and goal_time_tolerance here
        # # See http://docs.ros.org/en/api/control_msgs/html/action/FollowJointTrajectory.html

        # # Send the goal to the action client, and wait for it to finish.
        # self.action_client.send_goal(goal)
        # self.action_client.wait_for_result()
        # result = self.action_client.get_result()
        # print("Result ", result)
        """
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     0.0,
                                                     max_velocity_scaling_factor=vel)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                scene.removeCollisionObject("keepout")
                return 0 """

    def traj_playback(self, choice):
        """
        Function that plays back a saved trajectory
        """
        if choice == 1:
            file_name = 'table.yaml'
        elif choice == 2:
            file_name = 'cone.yaml'
        elif choice == 3:
            file_name = 'mug.yaml'
        elif choice == 4:
            file_name = 'sensor_array.yaml'
        elif choice == 5:
            file_name = 'feedback_mug_2.yaml'
        elif choice == 6:
            file_name = 'feedback_cone_3.yaml'
        with open(file_name, 'r') as user_file:
            traj = yaml.load(user_file)

        self.command_pub.publish("start")
        self.group.execute(traj)
        self.command_pub.publish("stop")


if __name__ == '__main__':
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    rospy.init_node('execute_path', anonymous=True)
    ## Instantiate a `ExecutePath` object
    motion = ExecutePath()

    print("\n\nMake sure all the trajectory .yaml files are in the directory this was launched from\n\n")


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ## Pause for 2 seconds after the motion is completed
        rospy.sleep(2)

        print("")
        print("====== Press 'Enter' to return to initial position =======")
        raw_input()
        motion.init_pose()

        ## 
        print("\nType the number of the trajectory to play back.\n1: Table\n2: Cone\n3: Mug\n4: Sensor Array\n5: Cone User Input\n6: Mug Uwer Input\n\n")
        choice = input("Selection: ")
        if choice == 1 or choice == 2 or choice == 3 or choice == 4 or choice == 5 or choice == 6:
            motion.traj_playback(choice)
        else:
            print("Enter 1, 2, 3, or 4")
        
        rate.sleep()