import time

# True only if a navigation request is in progress.
navPending = False
# True only if the pending navigation request needs to be cancelled.
needsCancel = False

# Navigate the robot to room 'name'.
# Set navPending to True when called, then set to False right before returning.
# Return 0 if navigation completed successfully.
# Return 1 if navigation failed.
# If needsCancel is True (value may change during execution),
# set needsCancel to False and return 2 as soon as possible.
def goTo(name):
    global navPending
    global needsCancel

    navPending = True

    if name == 'xy':
        navPending = False
        return 1

    for x in range(0, 10):
        if needsCancel:
            needsCancel = False
            navPending = False
            return 2
        time.sleep(1)

    navPending = False
    return 0

def goHome():
    time.sleep(5)
    return 0

def cancel():
    global navPending
    global needsCancel
    if navPending:
        needsCancel = True
