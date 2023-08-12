#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import pickle
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped


class EELocator:
    def __init__(self):
        self.listener = tf.TransformListener()

        # Initialize Subscribers
        self.start_sub = rospy.Subscriber('command', String, self.command_callback)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.vector_calculator)


        # Create a PointStamped message
        self.ps = PointStamped()
        self.ps.header.frame_id = '/gripper_link'
        self.ps.header.stamp = rospy.Time(0)  # You might need to adjust the time

        self.ps.point.x = 0.0
        self.ps.point.y = 0.0
        self.ps.point.z = -0.1

        self.command = None

        self.recorded_directional_vector = []
 
    def command_callback(self, msg):
        """
        A callback function that stores a String message that stops this node from
        publishing irridiance vectors.
        :param self: The self reference.
        :param msg: The String message type.
        """
        self.command = msg.data

        if self.command == "start":
            rospy.loginfo('Recording Poses.')

        elif self.command == "stop":
            ## Get user input for the filename
            filename = input("Enter a filename to save: ")
            # recorder.save(filename + '.pkl')

            with open(filename + '.pkl', 'wb') as f:
                pickle.dump(self.recorded_directional_vector, f)

            rospy.loginfo('Recorded {} end effector poses to {}.'.format(len(self.recorded_directional_vector), filename))
            rospy.loginfo('Recording finished.')

            self.recorded_directional_vector = []


    def vector_calculator(self, msg):

        if self.command == "start":
            trans, rot = self.find_ee_pose()
            new_ps = self.transform_point()
            tip_vector = [new_ps.point.x, new_ps.point.y, new_ps.point.z]
            self.recorded_directional_vector.append([trans, tip_vector])

    
            
    def find_ee_pose(self):
        """
        Function that finds the pose of the ee_link relative to the base_link frame
        :param self: The self reference.

        :return [trans, rot]: The Pose message type.
        """
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform( '/base_link', '/gripper_link',rospy.Time(0))
                return [trans,rot]
                if trans:
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
    
    def transform_point(self):
            try:
                new_ps = self.listener.transformPoint('/base_link', self.ps)#, rospy.Time(0))
                # trans = [round(p, 2) for p in trans]
                # rot = [round(r, 2) for r in rot]
                return new_ps
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

if __name__ == '__main__':
    rospy.init_node('ee_recorder')
    obj = EELocator()
    rospy.loginfo('Recorder node is running.')


    recorded_poses = []
    rospy.spin()
