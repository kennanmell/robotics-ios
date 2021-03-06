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
        print("create ActionRunner")
        # rospy.init_node('action_runner')
        wait_for_time()

        # get gripper
        self.gripper = fetch_api.Gripper()
        print("done with gripper")
        self.arm = fetch_api.Arm()
        print("done with arm")
        self.base = fetch_api.Base()
        print("done with base")
        self.reader = self.ArTagReader()
        self.markers = {}
        # get initial position of markers... it will continue updating in background
        reachable = False
        rospy.sleep(0.5)
        while len(self.reader.markers) == 0:
            # TODO[LOW PRIORITY]: implement looking for the AR tag by tilting its head up and down
            print("waiting for marker")
            self.base.turn(0.6)
            print("im about to sleep")
            rospy.sleep(1.5)
            print("i had a nap")
        print("found my marker!")

        markers = self.reader.markers
        print("Original marker pose is " + str(markers[0]))
        debug_marker_pose = PoseStamped(pose=markers[0].pose.pose)
        debug_marker_pose.header.stamp = rospy.Time.now()
        debug_marker_pose.header.frame_id = "/base_link"
        draw_debug_marker(debug_marker_pose, [0,1,0,0.5])
        original_pose_stamped = dot_poses(lookup_transform("/odom","/base_link"), markers[0].pose.pose)
        print("Adjusting orientation")
        number_of_turns = 0
        while not reachable:
            if number_of_turns > 5:
                print("Turned enough, I guess...")
                break
            while len(self.reader.markers) == 0:
                pass
            # m = markers[0]
            m = deepcopy(self.reader.markers[0])
            pose_stamped = PoseStamped(pose=m.pose.pose)
            pose_stamped.header.frame_id = "/base_link"
            pose_stamped.header.stamp = rospy.Time.now()
            turn = compute_turn(deepcopy(pose_stamped.pose))
            print("\tComputed turn:" + str(turn))
            if abs(turn) > 0.07:
                self.base.turn(turn)
                number_of_turns += 1
                rospy.sleep(1.5)
                print("\tExecuted turn")
            # markers = self.reader.markers
            if abs(compute_turn(m.pose.pose)) <= MIN_TURN_RADS:
                print("\tComputed turn was less than " + str(MIN_TURN_RADS) + ": " + str(abs(compute_turn(m.pose.pose))))
                print("...good enough")
                break
        print("Re-Orientation complete!")
        print("Adjusting forward position...")

        if self.arm.compute_ik(pose_stamped):
            print("\tMarker is already reachable")
            reachable = True
        else:
            reachable = False
            print("\tMarker is too far away from the robot")
            computed_forward = compute_dist(pose_stamped.pose) - 1
            print("\tComputed forward distance is " + str(computed_forward))
            self.base.go_forward(computed_forward)
            print("\tMoving forward by " + str(computed_forward))
        print("Moved to location")
        # move gripper
        #target_pose = PoseStamped(pose=original_pose_stamped.pose)
        #target_pose = PoseStamped(pose=markers[0].pose.pose)
        original_pose_stamped.pose.position.x -= 0.25
        original_pose_stamped.pose.orientation.x = 0
        original_pose_stamped.pose.orientation.y = 0.7
        original_pose_stamped.pose.orientation.z = 0
        original_pose_stamped.pose.orientation.w = -0.7
        target_pose = dot_poses(lookup_transform("/base_link", "/odom"), original_pose_stamped.pose)
        target_pose.header.frame_id = "/base_link"
        target_pose.header.stamp = rospy.Time.now()
        print("Marker pose after moving is " + str(target_pose))
        draw_debug_marker(target_pose, [0,0,1,0.5])
        if self.arm.compute_ik(target_pose):
            self.success = True
            print("reachable")
            # FINISHED: the gripper moves significantly slower by changing MoveGroup's
            #   MotionPlanRequest's max_velocity_scaling factor.
            print("move_to_pose result: " + str(self.arm.move_to_pose(target_pose)))
        else:
            print("not reachable :(")
            self.success = False

        # TODO[HARD]: get pre-recorded motion and apply that motion to start navigating the user
        # rospy.spin()


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
  rospy.init_node("lol")
  ar = ActionRunner()

if __name__ == '__main__':
  main()
