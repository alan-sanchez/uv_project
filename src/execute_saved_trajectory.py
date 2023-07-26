#!/usr/bin/env python

## Import modules
import sys
import rospy
import yaml 
import signal
import moveit_commander
import moveit_msgs.msg

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
        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")

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

    def traj_playback(self, choice):
        """
        Function that plays back a saved trajectory
        """
        if choice == 1:
            file_name = 'table.yaml'
        elif choice == 2:
            file_name = 'cone_b.yaml'
        elif choice == 3:
            file_name = 'mug_b.yaml'
        elif choice == 4:
            file_name = 'test8.yaml'
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
        print("\nType the number of the trajectory to play back.\n1: Table\n2: Cone\n3: Mug\n4: Sensor Array\n\n")
        choice = input("Selection: ")
        if choice == 1 or choice == 2 or choice == 3 or choice == 4:
            motion.traj_playback(choice)
        else:
            print("Enter 1, 2, 3, or 4")
        
        rate.sleep()