#!/usr/bin/env python

## Import modules
import sys
import rospy
import yaml 
import signal

from std_msgs.msg import String

class Stopper(object):
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
        ## Initialize Publisher
        self.command_pub = rospy.Publisher('/command', String, queue_size=10)



    def send_stop(self):
        ##
        self.command_pub.publish("stop")


if __name__ == '__main__':
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    rospy.init_node('stopper', anonymous=True)
    ## Instantiate a `ExecutePath` object
    cmd = Stopper()



    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ## Pause for 2 seconds after the motion is completed
        rospy.sleep(2)

        print("")
        print("====== Press 'Enter' to return to send stop command =======")
        raw_input()
        cmd.send_stop()

        ## 
       
        
        rate.sleep()