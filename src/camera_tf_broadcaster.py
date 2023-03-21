#!/usr/bin/env python2

## Import modules
import rospy
import math

## Import tf.transformations the change quaternion values to euler values
import tf.transformations

## The TransformStamped message is imported to broadcast a transform frame
from geometry_msgs.msg import TransformStamped

## Import StaticTransformBroadcaster to publish static transforms
from tf2_ros import StaticTransformBroadcaster

class Broadcaster():
    """
    This node publishes a child static frame in reference to their parent frame as below:
    parent -> camera            child -> base_link
    """
    def __init__(self):
        """
        A function that creates a broadcast node and publishes three new transform
        frames.
        :param self: The self reference.
        """
        ## Create a broadcast node
        self.br = StaticTransformBroadcaster()

        ## Create a stamped transform to broadcast
        self.camera = TransformStamped()

        ## Define parent and child frames
        self.camera.header.stamp = rospy.Time.now()
        self.camera.header.frame_id = 'gripper_link'
        self.camera.child_frame_id = 'camera_link'

        ## Set pose values to transform
        self.camera.transform.translation.x = -0.09
        self.camera.transform.translation.y =  0.0175
        self.camera.transform.translation.z = -0.07
        q = tf.transformations.quaternion_from_euler(0,math.pi/2,0)
        self.camera.transform.rotation.x = q[0]
        self.camera.transform.rotation.y = q[1]
        self.camera.transform.rotation.z = q[2]
        self.camera.transform.rotation.w = q[3]

        ## Create a stamped transform to broadcast
        self.uv_light = TransformStamped()

        ## Define parent and child frames
        self.uv_light.header.stamp = rospy.Time.now()
        self.uv_light.header.frame_id = 'gripper_link'
        self.uv_light.child_frame_id = 'uv_light_link'

        ## Set pose values to transform
        self.uv_light.transform.translation.x =  0
        self.uv_light.transform.translation.y =  0
        self.uv_light.transform.translation.z = -0.095
        q = tf.transformations.quaternion_from_euler(0,0,0)
        self.uv_light.transform.rotation.x = q[0]
        self.uv_light.transform.rotation.y = q[1]
        self.uv_light.transform.rotation.z = q[2]
        self.uv_light.transform.rotation.w = q[3]


        ## Publish transforms in a list
        self.br.sendTransform([self.camera, self.uv_light])


        ## Create rospy log message
        rospy.loginfo('Publishing TF frames. Use RViz to visualize')

if __name__ == '__main__':
    ## Initialize the node, and call it "tf2_broadcaster"
    rospy.init_node('camera_tf_broadcaster')

    ## Instantiate the `Broadcaster()` class
    Broadcaster()

    ## Give control to ROS.  This will allow the callback to be called whenever new
    ## messages come in.  If we don't put this line in, then the node will not work,
    ## and ROS will not process any messages
    rospy.spin()
