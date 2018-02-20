import fetch_api
import rospy

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
    if navPending:
        needsCancel = True
