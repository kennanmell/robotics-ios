#! /usr/bin/env python

import sys
import os
import pickle
from copy import deepcopy
import tf
import tf.transformations as tft
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Quaternion, Point
import rospy
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionGoal

from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import fetch_api
import rospy
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState
import actionlib
from math import atan, sqrt
from transformation_api import *
from visualization_msgs.msg import Marker


MIN_TURN_RADS = 0.15

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def compute_turn(pose):
    return atan(pose.position.y / pose.position.x)

def compute_dist(pose):
    return sqrt(pow(pose.position.x, 2) + pow(pose.position.y, 2))

def draw_debug_marker(pose, rgba=[0.0,0.5,0.5,0.5]):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.header = pose.header
    marker.pose = pose.pose
    marker.scale.x = 0.15
    marker.scale.y = 0.15
    marker.scale.z = 0.15
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.color.a = rgba[3]
    marker_publisher = rospy.Publisher('debug_marker', Marker, queue_size=10)
    rospy.sleep(0.5)
    marker_publisher.publish(marker)

class ActionRunner(object):
    def __init__(self):
        print("-------------STARTING PERCEPTION SEQUENCE-----------")
        ###################START SETUP###################
        print("create ActionRunner")
        # rospy.init_node('action_runner')
        wait_for_time()

        # initial setup of robot component that is needed
        print("setting up gripper, arm, base, and AR Tag reader")
        self.gripper = fetch_api.Gripper()
        print("\tdone with gripper")
        self.arm = fetch_api.Arm()
        print("\tdone with arm")
        self.base = fetch_api.Base()
        print("\tdone with base")
        self.reader = self.ArTagReader()
        print("\tdone with AR Tag reader")
        self.markers = {}
        print("")

        # sleeping to make sure all subscribers are all set up
        rospy.sleep(0.5)
        print("done setup!")
        ###################FINISHED SETUP###################

        ###################START FINDING AR TAG IN ROBOT'S VIEW###################
        print("START finding AR tag in robot's view")
        while len(self.reader.markers) == 0:
            print("\tattempting to find AR marker")
            self.base.turn(0.6)
            print("\tsleeping...")
            rospy.sleep(1.2)
        print("found marker on phone!!")
        print("------------------------")
        ###################FINISHED FINDING AR TAG IN ROBOT'S VIEW###################

        ###################START CENTERING AR TAG IN ROBOT'S VIEW###################
        markers = self.reader.markers
        print("Original marker pose is " + str(markers[0])) # prints the AR marker position

        # Creating debug marker for the original position of marker
        debug_marker_pose = PoseStamped(pose=markers[0].pose.pose)
        debug_marker_pose.header.stamp = rospy.Time.now()
        debug_marker_pose.header.frame_id = "/base_link"
        print("\tdrawing debug marker of the location of the ar tag...")
        draw_debug_marker(debug_marker_pose, [0,1,0,0.5])

        # set pose to be of frame odom... UTILIZED LATER FOR MOVING THE GRIPPER
        original_pose_stamped = dot_poses(lookup_transform("/odom","/base_link"), markers[0].pose.pose)

        # start rotating to center ar tag within view
        print("Adjusting orientation")
        number_of_turns = 0 # if centering takes longer than 5 turns, the centering algorithm gives up
        # assume that the AR tag is not reachable at first
        reachable = False
        while not reachable:
            if number_of_turns > 5:
                print("\tTurned enough, I guess...")
                break
            # If for some odd reason, we lost track of the marker, just continue to wait til the marker is in sight
            while len(self.reader.markers) == 0:
                print("\tLost marker... attempting to find marker...")
                rospy.sleep(0.1)
            # gets the pose of the marker and compute the amount of turn required to center
            m = deepcopy(self.reader.markers[0])
            pose_stamped = PoseStamped(pose=m.pose.pose)
            pose_stamped.header.frame_id = "/base_link"
            pose_stamped.header.stamp = rospy.Time.now()
            turn = compute_turn(deepcopy(pose_stamped.pose))
            print("\tComputed turn:" + str(turn))

            # if the turn is big neough to actually turn then turn
            if abs(turn) > 0.07:
                self.base.turn(turn)
                number_of_turns += 1
                rospy.sleep(1.5)
                print("\tExecuted turn")
            # if the turn left over is less than a set minimum turn, then we are done
            if abs(compute_turn(m.pose.pose)) <= MIN_TURN_RADS:
                print("\tComputed turn was less than " + str(MIN_TURN_RADS) + ": " + str(abs(compute_turn(m.pose.pose))))
                print("\t...good enough")
                break
        print("Re-Orientation complete!")
        print("------------------------")
        ###################FINISHED CENTERING AR TAG IN ROBOT'S VIEW###################

        ###################START POSITIONING THE AR TAG SUCH THAT IT IS REACHABLE###################
        print("Adjusting forward position...")
        # readjusting the positioning of the AR tag such that the gripper will not hit the user.
        pose_stamped.pose.position.x -= 0.25
        pose_stamped.pose.position.y -= 0.05
        pose_stamped.pose.orientation.x = 0
        # readjusting the positioning of the AR tag such that the gripper will be upright.
        pose_stamped.pose.orientation.y = 0.7
        pose_stamped.pose.orientation.z = 0
        pose_stamped.pose.orientation.w = -0.7
        # initializing two post_stamped
        original = deepcopy(pose_stamped)
        forward = deepcopy(pose_stamped)
        backward = deepcopy(pose_stamped)
        working = None
        print("checking if moving is required...")
        iteration_count = 0
        while True: # continues to loop until a position is reachable
            # the arm is just not going to reach the tag
            if iteration_count > 40:
                print("The tag might not be reachable by the robot")
                return
            if self.arm.compute_ik(foward):
                working = forward
                break
            elif self.arm.compute_ik(backward):
                working = backward
                break
            else:
                print("\tMarker is not reachable! Readjusting by moving the marker closer or further")
                # update the marker state
                forward.pose.position.x += 0.05
                backward.pose.position.x -= 0.05
                forward.header.stamp = rospy.Time.now()
                backward.header.stamp = rospy.Time.now()
            iteration_count += 1
            rospy.sleep(0.1)
        # calculates the amount the robot needs to move forward for compute_ik to succeed
        computed_forward = working.pose.position.x - original.pose.position.x
        # make minor adjustment to computed_foward just in case the compute_ik results in a too percise measurement
        if computed_foward < 0:
            computed_foward -= 0.02
            print("\trobot must move backward " + str(computed_forward) + " meters")
        elif computed_foward > 0:
            computed_foward += 0.02
            print("\trobot must move forward " + str(computed_forward) + " meters")
        else:
            print("\tNo adjustment is needed on the robot")
        print("\tMoving vertically: " + str(computed_forward) + "meters")
        self.base.go_forward(computed_forward)
        print("\tDone moving vertically: " + str(computed_forward) + "meters")

        ###################CODE NOT NEEDED I THINK?###############
        # if self.arm.compute_ik(pose_stamped):
        #     print("\tMarker is already reachable")
        #     reachable = True
        # else:
        #     reachable = False
        #     print("\tMarker is too far away from the robot")
        #     computed_forward = compute_dist(pose_stamped.pose) - .85
        #     print("\tComputed forward distance is " + str(computed_forward))
        #     max_forward = 0.05 if computed_forward > 3.2 else computed_forward
        #     self.base.go_forward(max_forward)
        #     print("\tMoving forward by " + str(max_forward))
        # marker is now reachable
        print("DONE Adjusting forward position")
        print("------------------------")
        ###################FINISHED POSITIONING THE AR TAG SUCH THAT IT IS REACHABLE###################

        ###################START MOVING GRIPPER TO THE AR TAG###################
        print("Starting move grip sequence")
        # readjusting the positioning of the AR tag such that the gripper will not hit the user.
        original_pose_stamped.pose.position.x -= 0.25
        original_pose_stamped.pose.position.y -= 0.05
        original_pose_stamped.pose.orientation.x = 0
        # readjusting the positioning of the AR tag such that the gripper will be upright.
        original_pose_stamped.pose.orientation.y = 0.7
        original_pose_stamped.pose.orientation.z = 0
        original_pose_stamped.pose.orientation.w = -0.7
        # putting the gripper pose back to base_link
        target_pose = dot_poses(lookup_transform("/base_link", "/odom"), original_pose_stamped.pose)
        target_pose.header.frame_id = "/base_link"
        target_pose.header.stamp = rospy.Time.now()
        print("Marker pose we're trying to put the gripper at is: " + str(target_pose))
        print("drawing debug marker of the location we will be reaching for...")
        draw_debug_marker(target_pose, [0,0,1,0.5])
        # computing reachability
        if self.arm.compute_ik(target_pose):
            self.success = True
            print("reachable")
            # FINISHED: the gripper moves significantly slower by changing MoveGroup's
            #   MotionPlanRequest's max_velocity_scaling factor.
            print("move_to_pose result: " + str(self.arm.move_to_pose(target_pose)))
        else:
            print("not reachable :(")
            self.success = False
        ###################FINISHED MOVING GRIPPER TO THE AR TAG###################
        print("-------------DONE WTIH PERCEPTION SEQUENCE-----------")

    class ArTagReader(object):
        def __init__(self):
            self.markers = []
            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback)
        def callback(self, msg):
            self.markers = msg.markers
"""
Executes the action from action_saver.py

Starts the arm controller.
Uses undocumented feature of the Fetch robot. The fetch robot API exposes
    an action named /query_controller_states of type
    robot_controllers_msgs/QueryControllerStates.
Creates an action client to use this action.
"""
def runner():
    ar = ActionRunner()
    return ar.success

def main():
  #  start the arm controller
  # wtf
  ar = ActionRunner()

if __name__ == '__main__':
  main()
