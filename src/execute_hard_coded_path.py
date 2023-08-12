#!/usr/bin/env python

## Import modules
import sys
import rospy
import yaml 
import json
import signal
import moveit_commander
import moveit_msgs.msg

## Import message types and other python librarires
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from std_msgs.msg import String
from geometry_msgs.msg import  Pose, Point, Quaternion

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
        self.group.set_end_effector_link("gripper_link") #use gripper_link if it is planar disinfection

        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        ## Initialize waypoints variable
        self.waypoints = None

        ##
        self.velocity = 0.16 #0.278 #0.19 #0.571
 
    def plan_cartesian_path(self):
        """
        
        """
        ################### Cartesian waypoints for table #####################
        # waypoints = [Pose(Point(0.725,  0.60, 1.12),Quaternion(0.000, 0.0, 0, 1)),#0.131, 0, 0, 0.991)),
        #              Pose(Point(0.725,  0.50, 1.12),Quaternion(0.000, 0.0, 0, 1)),
        #              Pose(Point(0.725,  0.40, 1.12),Quaternion(0.000, 0.0, 0, 1)),
        #              Pose(Point(0.725,  0.25, 1.12),Quaternion(0.000, 0.0, 0, 1)),
        #              Pose(Point(0.725,  0.00, 1.12),Quaternion(0.000, 0.0, 0, 1)),
        #              Pose(Point(0.725, -0.25, 1.12),Quaternion(0.000, 0.0, 0, 1)),
        #              Pose(Point(0.725, -0.50, 1.12),Quaternion(0.000, 0.0, 0, 1)),
        #              Pose(Point(0.725, -0.60, 1.12),Quaternion(0.000, 0.0, 0, 1)),#-0.131, 0, 0, 0.991)),
        #              ]
        waypoints = [Pose(Point(0.725,  -0.5, 1.17),Quaternion(0,0,0,1))]

        ################### Cartesian waypoints for cone #####################
        # waypoints = [Pose(Point(0.7,  0.3,  1.266), Quaternion(-0.36, -0.29, -0.21, 0.86)),
        #              Pose(Point(0.7, -0.3,  1.266), Quaternion( 0.36, -0.29,  0.21, 0.86)),
        #              Pose(Point(0.7, -0.3,  1.157), Quaternion( 0.36, -0.29,  0.21, 0.86)),
        #              Pose(Point(0.54,-0.0,  1.157), Quaternion( 0.06,  0.43, -0.25, -0.86)),
        #              Pose(Point(0.7,  0.3,  1.157), Quaternion(-0.36, -0.29, -0.21, 0.86)),
        #              Pose(Point(0.7,  0.3,  1.058), Quaternion(-0.36, -0.29, -0.21, 0.86)),
        #              Pose(Point(0.54,-0.0,  1.060), Quaternion( 0.06,  0.43, -0.25, -0.86)),
        #              Pose(Point(0.7, -0.3,  1.058), Quaternion( 0.36, -0.29,  0.21, 0.86)),
        #              Pose(Point(0.65,-0.17, 1.111), Quaternion( 0.14, -0.19,  0.21, 0.95)),
        #              Pose(Point(0.64, 0.22, 1.111), Quaternion(-0.14, -0.19, -0.21, 0.95)),
        #            ]

        # waypoints = [Pose(Point(0.85,  0.0, 1.42),Quaternion(0.00, 0.00, 0.00,  1.00)),]

        ################### Cartesian waypoints for mug #####################
        # waypoints = [Pose(Point(0.846, 0.413, 0.977),Quaternion(-0.54, -0.01, -0.03, 0.84)),
        #              Pose(Point(0.841, 0.286, 1.171),Quaternion(-0.33, -0.01, -0.03, 0.94)),
        #              Pose(Point(0.836, 0.072, 1.211),Quaternion(-0.02, -0.0, -0.03, 1.0)),
        #              Pose(Point(0.836,-0.101, 1.21),Quaternion(-0.02, -0.0, -0.03, 1.0)),
        #              Pose(Point(0.658, -0.261, 1.063),Quaternion(-0.39, 0.17, 0.19, -0.88)),
        #              Pose(Point(0.594, 0.069, 1.057),Quaternion(0.06, 0.36, 0.14, -0.92)),
        #              Pose(Point(0.648, 0.288, 1.08),Quaternion(-0.33, -0.27, -0.11, 0.9)),

        #             #  Pose(Point(0.85, -0.10, 1.2),Quaternion(0.00, 0.00, 0.00,  1.00)),
        #             #  Pose(Point(0.80, -0.10, 1.2),Quaternion(0.00, 0.00, 0.00,  1.00)),
        #             #  Pose(Point(0.80,  0.00, 1.2),Quaternion(0.00, 0.00, 0.00,  1.00)),
        #             #  Pose(Point(0.80,  0.10, 1.2),Quaternion(0.00, 0.00, 0.00,  1.00)),        
        #            ]


        ################### Cartesian waypoints for square region #####################
        # waypoints = [Pose(Point(0.846, 0.413, 0.977),Quaternion(-0.54, -0.01, -0.03, 0.84)),
        #              Pose(Point(0.841, 0.286, 1.171),Quaternion(-0.33, -0.01, -0.03, 0.94)),
        #              Pose(Point(0.836, 0.072, 1.211),Quaternion(-0.02, -0.0, -0.03, 1.0)),
        #              

        waypoints = [Pose(Point(.75,  0.2, 1.2),Quaternion(0.00, 0.00, 0.00,  1.00)),]

       

        (plan, fraction) = self.group.compute_cartesian_path(waypoints, # waypoints to follow
                                                             0.1,           # eef_step
                                                             0.00)          # jump_threshold
    
        plan = self.group.retime_trajectory(self.robot.get_current_state(),
                                            plan,
                                            velocity_scaling_factor = self.velocity,
                                            )
        
        # print("\n\nEnter 1 to record trajectory \nEnter 2 if not \nEnter 3 to cancel \n")
        # save_traj = input("Choose an option: ")

        # if save_traj == 1:
        #     traj_name = raw_input('Name the trajectory: ')
        #     with open(traj_name + '.yaml', 'w') as outfile:
        #         yaml.dump(plan, outfile, default_flow_style=True)

        # elif save_traj == 3:
        #     return None

        return plan
   

    def execute_plan(self, plan):
        """
        
        """
        ## Publish string command to initiate functions in other nodes
        self.command_pub.publish("start")

        self.group.execute(plan, wait=True)

        ## Publish string command to stop UV accumulation mapping from other nodes
        self.command_pub.publish("stop") 


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


if __name__ == '__main__':
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    rospy.init_node('execute_path', anonymous=True)
    ## Instantiate a `ExecutePath` object
    motion = ExecutePath()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ## Pause for 2 seconds after the motion is completed
        rospy.sleep(2)

        # print("")
        # print("====== Press 'Enter' to return to initial position =======")
        # raw_input()
        # motion.init_pose()
        
        ## Wait for user input before generating new disinfection region
        ## Print out instructions for user input to get things started
        print("")
        print("====== Press 'Enter' to execute hard-coded path =======")
        raw_input()
        plan = motion.plan_cartesian_path()
        
        if plan != None:
            motion.execute_plan(plan)
        
        rate.sleep()

