#!/usr/bin/env python

## Import modules
import rospy

## Import message types and other python libraries
import actionlib
from robot_controllers_msgs.msg import QueryControllerStatesAction, \
                                       QueryControllerStatesGoal, \
                                       ControllerState

class ArmControl:
    '''
    Class for controlling mid-level operations of the arm.
    '''
    def __init__(self):
        '''
        A function that relaxes the manipulator.
        :param self: The self reference.
        '''
        controller_states = "/query_controller_states"

        self._controller_client = actionlib.SimpleActionClient(controller_states,QueryControllerStatesAction)
        self._controller_client.wait_for_server()

        self._gravity_comp_controllers = ["arm_controller/gravity_compensation"]

        self._non_gravity_comp_controllers = list()
        self._non_gravity_comp_controllers.append("arm_controller/follow_joint_trajectory")
        self._non_gravity_comp_controllers.append("arm_with_torso_controller/follow_joint_trajectory")


    def relax_arm(self):
        '''
        Turns on gravity compensation controller and turns
        off other controllers
        '''
        goal = QueryControllerStatesGoal()

        for controller in self._gravity_comp_controllers:
            state = ControllerState()
            state.name = controller
            state.state = state.RUNNING
            goal.updates.append(state)

        for controller in self._non_gravity_comp_controllers:
            state = ControllerState()
            state.name = controller
            state.state = state.STOPPED
            goal.updates.append(state)

        self._controller_client.send_goal(goal)

    def un_relax_arm(self):
        '''
        Turns on gravity compensation controller and turns
        on other controllers
        '''
        goal = QueryControllerStatesGoal()

        for controller in self._non_gravity_comp_controllers:
            state = ControllerState()
            state.name = controller
            state.state = state.RUNNING
            goal.updates.append(state)

        # for controller in self._gravity_comp_controllers:
        #     state = ControllerState()
        #     state.name = controller
        #     state.state = state.STOPPED
        #     goal.updates.append(state)

        self._controller_client.send_goal(goal)
   
if __name__ == '__main__':
    ## Initialize the `relax_arm_control` node
    rospy.init_node('relax_arm_control')

    ## Instantiate the `ArmControl()` object
    obj = ArmControl()

    ## Run the `un_relax_arm()` method
    obj.un_relax_arm()

    ## Notify user that they can move the arm
    rospy.loginfo("Relaxed arm node activated. You can now move the manipulator")
    print()
    rospy.loginfo("Type Ctrl + C when you are done recording")
    rospy.spin()