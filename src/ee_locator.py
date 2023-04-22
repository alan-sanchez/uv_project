#!/usr/bin/env python  

## Import modules
import rospy
import tf

class EELocator():
    """
    A Class that computes the linear and angular velocity of the end effector. It then
    publishes that information as a TwistStamped message.  
    """
    def __init__(self):
        """
        Initialize the pulisher and other variables
        :param self: The self reference. 
        """
        
        ## Initialize transform listener
        self.listener = tf.TransformListener()

       
    def ee_pose(self):
        """
        Function thatfinds the pose of the `gripper_link` relative to the `base_link` frame.
        :param self: The self reference.

        :return [trans, rot]: The pose message type.
        """
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform( '/base_link', '/gripper_link',rospy.Time(0))
                trans = [round(p,3) for p in trans]
                rot = [round(r,2) for r in rot]
                return [trans,rot]

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        
if __name__ == '__main__':
    ## Initialize `arm_twist_cmds` node
    rospy.init_node('ee_locator')

    ## Instantiate the `ArmTwistcmds` object
    obj = EELocator()

    ## Set rospy rate
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        ##
                # print()

        raw_input("Press enter if you want the pose of the end effector.")
        ## Call on the `vel_calculator()` method
        val = obj.ee_pose()
        print(val)
        print("")

        ## Run the `sleep()` method 
        rate.sleep()