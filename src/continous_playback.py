#!/usr/bin/env python

import signal
import sys
import time
import rospy
import json
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import  Pose, Point, Quaternion
from std_msgs.msg import String
from moveit_msgs.msg import RobotTrajectory
import yaml


VELOCITY = 0.19
DIST_THRESHOLD = 0.2

def signal_handler(signal, frame):
    global interrupted
    interrupted = True

signal.signal(signal.SIGINT, signal_handler)

interrupted = False


def save_movement(p_x, p_y, p_z, q_x, q_y, q_z, q_w, time_in_seconds):
    rospy.loginfo('writing to file')
    file_name = raw_input('\nname your movement file: ')
    dictionary = {
        "sampling_rate": time_in_seconds,
        "positions": [{"x": p_x}, {"y": p_y}, {"z": p_z}],
        "orientations": [{"x": q_x}, {"y": q_y}, {"z": q_z}, {"w": q_w}]
    }
    json_object = json.dumps(dictionary, indent=4)
    with open(file_name + '.json', 'w') as outfile:
        outfile.write(json_object)

def playback():
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm_with_torso")
    group.set_end_effector_link("gripper_link")
    command_pub = rospy.Publisher('/command', String, queue_size=10)
    group.set_planning_time(10.0)
    #open the file to be read from and load into json object
    file_name = raw_input('\nEnter the name of the movement file you wish to replay: ')
    with open(file_name + '.json') as user_file:
        poses_object = json.load(user_file)
    # get the arrays of poses to move to and times to hold
    p_x = poses_object["positions"][0]["x"]
    p_y = poses_object["positions"][1]["y"]
    p_z = poses_object["positions"][2]["z"]
    q_x = poses_object["orientations"][0]["x"]
    q_y = poses_object["orientations"][1]["y"]
    q_z = poses_object["orientations"][2]["z"]
    q_w = poses_object["orientations"][3]["w"]

    sampling_rate = poses_object["sampling_rate"]
    # Reconstruct poses from JSON file and store them in a list to be given to movegroup
    waypoints = []
    for idx in range(len(p_x)):
        new_pose = Pose(Point(p_x[idx], p_y[idx], p_z[idx]),Quaternion(q_x[idx], q_y[idx], q_z[idx], q_w[idx]))
        waypoints.append(new_pose)
    
    fraction = 0
    count = 0
    while fraction < 0.9:
        (plan, fraction) = group.compute_cartesian_path(waypoints, # waypoints to follow
                                                        0.01,      # eef_step
                                                        0.00)      # jump_threshold
        count +=1
        if count > 100:
            break
    
    plan = group.retime_trajectory(robot.get_current_state(),
                                        plan,
                                        velocity_scaling_factor = 0.2,
                                        acceleration_scaling_factor = .2,
                                        )
    command_pub.publish("start")
    if fraction > 0.9:
        group.execute(plan, wait=True)
    else:
        print("nope", fraction)# rospy.WARN("Could not plan the cartesian path")
    command_pub.publish("stop")

    # Save movement trajectory so that it doesnt have to be recalculated later
    save_traj = input("Save this trajectory? Enter 1 for yes or 2 for no: ")
    if save_traj == 1:
        traj_name = raw_input('Name the trajectory: ')
        with open(traj_name + '.yaml', 'w') as outfile:
            yaml.dump(plan, outfile, default_flow_style=True)


def traj_playback():
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("arm_with_torso")
    group.set_end_effector_link("gripper_link")
    command_pub = rospy.Publisher('/command', String, queue_size=10)

    file_name = raw_input('\nEnter the name of the trajcetory file you wish to replay: ')
    with open(file_name + '.yaml', 'r') as user_file:
        traj = yaml.load(user_file)

    command_pub.publish("start")
    group.execute(traj, wait=True)
    command_pub.publish("stop")


if __name__ == '__main__':
    rospy.init_node("state_playback")

    # dont think this is being used. delete if confirmed not needed
    robot = moveit_commander.RobotCommander()

    # init the planning scene
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    
    # outer loop controlling create movement or play movement
    while(True):
        print(robot.get_group_names())
        print("\n\nEnter 1 to record a movement\nEnter 2 to playback a pose\nEnter 3 to playback from a pre-calculated trajectory\nEnter 4 to quit\n")
        control_selection = input("Choose an option: ")

        ## sub loop controlling add a movement feature
        if control_selection == 1:
            ## init the arrays that will be used to store pose components
            p_x = []
            p_y = []
            p_z = []
            q_x = []
            q_y = []
            q_z = []
            q_w = []
            group_name = "arm_with_torso"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            move_group.set_end_effector_link("gripper_link")
            sampling_rate = input("Enter your sampling rate in Hz (Recommended 0.25 or 0.5): ")
            rate_in_seconds = 1.0 / sampling_rate

            print("\nRecording movement...\n")
            print("\nPress ctrl+c to stop recording and save to file\n")
            # Get the intial pose
            ee_pose = move_group.get_current_pose("gripper_link")
            p_x.append(round(ee_pose.pose.position.x, 2))
            p_y.append(round(ee_pose.pose.position.y, 2))
            p_z.append(round(ee_pose.pose.position.z, 2))
            q_x.append(round(ee_pose.pose.orientation.x, 2))
            q_y.append(round(ee_pose.pose.orientation.y, 2))
            q_z.append(round(ee_pose.pose.orientation.z, 2))
            q_w.append(round(ee_pose.pose.orientation.w, 2))

            while(True) :

                ## Save a movement at defined rate
                # Get end effector pose. Place its attributes into arrays for storage in a JSON file
                ee_pose = move_group.get_current_pose("gripper_link")
                p_x.append(round(ee_pose.pose.position.x, 2))
                p_y.append(round(ee_pose.pose.position.y, 2))
                p_z.append(round(ee_pose.pose.position.z, 2))
                q_x.append(round(ee_pose.pose.orientation.x, 2))
                q_y.append(round(ee_pose.pose.orientation.y, 2))
                q_z.append(round(ee_pose.pose.orientation.z, 2))
                q_w.append(round(ee_pose.pose.orientation.w, 2))
                # For saving poses at correct rate. Ensures data set is not too large.
                time.sleep(rate_in_seconds)

                ## keyboard interupt. stop collecting data and save to file
                if interrupted:
                    print("\nSaving movement...\n")
                    save_movement(p_x[1:], p_y[1:], p_z[1:], q_x[1:], q_y[1:], q_z[1:], q_w[1:], rate_in_seconds)   # removes first pose from list since first pose can be bugged
                    interrupted = False
                    break
            
        ## play back a movement from file
        elif control_selection == 2:
            playback()

        elif control_selection == 3:
            traj_playback()

        elif control_selection == 4:
            break

        else:
            print("\nInvalid selection\n")
