import fetch_api
import time
import os
import rospy
import pickle
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionGoal
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header

class VelocityCallback(object):
    def __init__(self):
        print 'initialized'
        self.motion = True
        rospy.Subscriber('cmd_vel', Twist, self.callback)

    def callback(self, msg):
        print 'callback'
        self.motion = msg.linear.x != 0 or msg.linear.y != 0 or msg.linear.z != 0 or msg.angular.x != 0 or msg.angular.y != 0 or msg.angular.z != 0


# True only if a navigation request is in progress.
navPending = False
# True only if the pending navigation request needs to be cancelled.
needsCancel = False

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

# Navigate the robot to room 'name'.
# Set navPending to True when called, then set to False right before returning.
# Return 0 if navigation completed successfully.
# Return 1 if navigation failed.
# If needsCancel is True (value may change during execution),
# set needsCancel to False and return 2 as soon as possible.
def goTo(name):
    # TODO: Implement this.
    global navPending
    global needsCancel
    pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
    navPending = True
    if name == 'lower torso':
        torso = fetch_api.Torso()
        torso.set_height(0.0)
        navPending = False
        return 0
    elif name == 'raise torso':
        torso = fetch_api.Torso()
        torso.set_height(0.4)
        navPending = False
        return 0
    else:
        if not os.path.exists('pickle/' + name):
            print 'No such pose \'' + name + '\''
            navPending = False
            return 1
        else:
            loadfile = open('pickle/' + name, 'rb')
            stampedCoPose = pickle.load(loadfile)
            loadfile.close()
            # TODO: Figure out why this goal doesn't cause any motion
            mbagoal = MoveBaseActionGoal()
            mbgoal = MoveBaseGoal()
            mbgoal.target_pose = stampedCoPose #potential issue here
            mbagoal.goal = mbgoal
            #mbagoal.header =
            #mbagoal.goal_id =
            pub.publish(mbagoal)
            # END TODO
            velCbk = VelocityCallback()
            while velCbk.motion:
                print 'waiting'
                time.sleep(1)
            navPending = False
            return 0
    navPending = False
    return 1

# Navigate the robot to its default location.
# Set navPending to True when called, then set to False right before returning.
# Return 0 if navigation completed successfully.
# Return 1 if navigation failed.
# If needsCancel is True (value may change during execution),
# set needsCancel to False and return 2 as soon as possible.
def goHome():
    # TODO: Implement this.
    global navPending
    global needsCancel
    return 1

# Requests that navigation be cancelled. Do not modify.
def cancel():
    global navPending
    global needsCancel
    pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
    pub.publish(GoalID())
    if navPending:
        needsCancel = True

def find():
    global navPending
    global needsCancel

    navPending = True

    navPending = False
    return 0

def cancelFind():
    global navPending
    global needsCancel
    if navPending:
        needsCancel = True
        
