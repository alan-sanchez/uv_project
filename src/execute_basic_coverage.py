#!/usr/bin/env python

## Import modules
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import tf.transformations


## Import message types and other python librarires
from threading import Thread
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene, RobotTrajectory
from moveit_python import MoveGroupInterface, PlanningSceneInterface
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
        self.start_pub = rospy.Publisher('start', String, queue_size=10)
        self.stop_pub  = rospy.Publisher('stop',  String, queue_size=10)

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

        ## This creates objects in the planning scene that mimic the table
        ## If these were not in place gripper could hit the table
        self.planning_scene = PlanningSceneInterface("base_link")
        
    def plan_cartesian_path(self):
        ## Cartesian Paths
        waypoints = [Pose(Point(0.75,  0.20, 1.2),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.75,  0.00, 1.2),Quaternion(0.000, 0.0, 0, 1)),
                     Pose(Point(0.75, -0.20, 1.2),Quaternion(0.000, 0.0, 0, 1)),
                     ]

        (plan, fraction) = self.group.compute_cartesian_path(waypoints, # waypoints to follow
                                                             0.1,       # eef_step
                                                             0.00)      # jump_threshold

        plan = self.group.retime_trajectory(self.robot.get_current_state(),
        plan,
        velocity_scaling_factor = 0.2,
        )

        return plan

    def execute_plan(self, plan):
        # Publish string command to initiate functions in other nodes
        self.start_pub.publish("start")

        self.group.execute(plan, wait=True)
        ## print(time.time()-start_time)

        # Publish string command to stop UV accumulation mapping from other nodes
        self.stop_pub.publish("stop")

    def init_pose(self, vel = 0.2):
        """
        Function that sends a joint goal that moves the Fetch's arm and torso to
        the initial position.
        :param self: The self reference.
        :param vel: Float value for arm velocity.
        """
        ## Set the velocity for Joint trajectory goal
        self.group.set_max_velocity_scaling_factor(vel)

        ## List of joint positions for the initial pose
        joints = [.15, 1.41, 0.30, -0.22, -2.25, -1.56, 1.80, -0.37,]
        plan = self.group.go(joints, wait=True)



if __name__ == '__main__':
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    rospy.init_node('execute_path', anonymous=True)

    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ## Wait for user input before generating new disinfection region
        ## Print out instructions for user input to get things started
        print("")
        print("====== Press 'Enter' to execute basic motion =======")
        raw_input()

        ## Instantiate a `ExecutePath` object
        motion = ExecutePath()
        plan = motion.plan_cartesian_path()
        motion.execute_plan(plan)
        motion.init_pose()
        rate.sleep()

