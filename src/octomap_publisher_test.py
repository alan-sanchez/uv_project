#!/usr/bin/env python

# Import modules
import rospy

# Import message types and other python libraries.
from octomap_msgs.msg import Octomap

class OctmapPublisher:
    """
    A class that generates a velocity ratio for the subscribed waypoints.
    """
    def __init__(self):
        """
        Initialize subscriber and publisher
        :param self: The self reference.
        """
        # Initialize subscriber
        self.oct_sub = rospy.Subscriber('/octomap_binary', Octomap, self.callback)

        # Intialize Publisher
        self.oct_pub  = rospy.Publisher('/new_oct_binary', Octomap, queue_size=10)

    def callback(self,msg):
        """
        Generate
        :param self: The self reference.
        :param msg: The PoseArray message type.
        """
        self.oct_pub.publish(msg)

if __name__ == '__main__':
    # Initialize velocity_regulator node
    rospy.init_node('octomap_publisher_test', anonymous=True)

    # Instantiate the VelocityRegulator class
    OctmapPublisher()
    rospy.spin()