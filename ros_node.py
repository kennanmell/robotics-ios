import fetch_api
import time
import os
import rospy
import pickle
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseActionGoal
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from std_msgs.msg import Header, String

class PressureCallback(object):
    def __init__(self):
        self.goal = None
        self.moving = False
        rospy.Subscriber('pressure', String, self.callback)

    def callback(self, msg):
        global pub
        global cancelPub
        if msg.data == 'cancel':
            print 'got cancel'
            self.moving = False
            cancel()
        else:
            # msg == 'start'
            print 'got start'
            if not self.moving and self.goal is not None:
                self.moving = True
                sendNavGoal(self.goal)

class VelocityCallback(object):
    def __init__(self):
        print 'initialized'
        self.motion = True
        rospy.Subscriber('move_base/status', GoalStatusArray, self.callback)

    def callback(self, msg):
        self.motion = len(msg.status_list) > 0 and (msg.status_list[-1].status != 3 and msg.status_list[-1].status != 4 and msg.status_list[-1].status != 5)
        print self.motion
        print msg.status_list[-1].status


# True only if a navigation request is in progress.
navPending = False
# True only if the pending navigation request needs to be cancelled.
needsCancel = False

pub = None
cancelPub = None
callback = None
pressureCallback = None

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def setup():
    global pub
    global callback
    global cancelPub
    global pressureCallback

    pub = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    cancelPub = rospy.Publisher('move_base/cancel', GoalID, queue_size = 10)
    #rospy.init_node('amcl', anonymous=True)
    rospy.init_node('ios2')
    callback = VelocityCallback()
    pressureCallback = PressureCallback()

def sendNavGoal(name):
    global navPending
    global needsCancel
    global callback
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
            mbgoal = MoveBaseGoal()
            mbgoal.target_pose.header.frame_id = 'map'
            mbgoal.target_pose.header.stamp = rospy.Time().now()
            mbgoal.target_pose.pose = stampedCoPose.pose #potential issue here
            #mbagoal.goal_id.id = 'ios'
            pub.send_goal(mbgoal)
            # END TODO
 

# Navigate the robot to room 'name'.
# Set navPending to True when called, then set to False right before returning.
# Return 0 if navigation completed successfully.
# Return 1 if navigation failed.
# If needsCancel is True (value may change during execution),
# set needsCancel to False and return 2 as soon as possible.
def goTo(name):
    global pressureCallback
    pressureCallback.goal = name
    rospy.sleep(2)
    while callback.motion:
        print 'waiting'
        rospy.sleep(1)
    navPending = False
    pressureCallback.goal = None
    return 0

# Navigate the robot to its default location.
# Set navPending to True when called, then set to False right before returning.
# Return 0 if navigation completed successfully.
# Return 1 if navigation failed.
# If needsCancel is True (value may change during execution),
# set needsCancel to False and return 2 as soon as possible.
def goHome():
    # TODO: Implement this.
    return goTo('entrance')

# Requests that navigation be cancelled. Do not modify.
def cancel():
    global navPending
    global needsCancel
    global cancelPub
    goal = GoalID()
    #goal.id = 'ios'
    cancelPub.publish(goal)
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
        
