import socket
import sys
from urllib2 import urlopen
from threading import Thread
import imp

try:
    imp.find_module('rospy')
    import rospy
    from ros_node import goTo, goHome, cancel, find, cancelFind
    rospy.init_node('ios_app')
    while rospy.Time().now().to_sec() == 0:
        pass
except ImportError:
    print 'rospy not installed... using dummy navigation'
    from dummy_ros_node import goTo, goHome, cancel, find, cancelFind

pair = 1
pairSucceeded = 2
pairFailed = 3
unpair = 4
speak = 5
speakDone = 6
goto = 7
gotoDone = 8
gotoFailed = 9
update = 10
kill = 11
robotPair = 12
robotPairSucceeded = 13
robotPairFailed = 14
ping = 19
cancelGoto = 20
cancelGotoSucceeded = 21
goHomeReq = 22
goHomeDone = 23
findMe = 25
findMeSucceeded = 26
findMeFailed = 27
cancelFindMe = 28

def handleFind(sock):
    findMeSucceeded = 26
    findMeFailed = 27

    result = find()
    if result == 0:
        print 'find succeeded'
        sock.sendall(findMeSucceeded)
    else:
        print 'find failed'
        sock.sendall(findMeFailed)

def handleNav(sock, name):
    gotoDone = 8
    gotoFailed = 9
    cancelGotoSucceeded = 21

    if name == None:
        # Go home
        result = goHome()
        if result == 0:
            print 'successfully went home'
            sock.sendall(chr(goHomeDone))
        else:
            print 'failed to go home'
            sock.sendall(chr(gotoFailed))
        return

    result = goTo(name)
    if result == 0:
        print 'nav succeeded'
        sock.sendall(chr(gotoDone))
    elif result == 1:
        print 'nav failed'
        sock.sendall(chr(gotoFailed))
    elif result == 2:
        print 'nav cancelled'
        sock.sendall(chr(cancelGotoSucceeded))

if len(sys.argv) != 3:
    print "Usage: python", sys.argv[0], "<server-ip> <server-portno>"
    exit()

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = (sys.argv[1], int(sys.argv[2]))
#server_address = ('localhost', int(sys.argv[1]))
try:
    sock.connect(server_address)
except:
    print 'unable to connect to server'
    exit()

print 'connected to server %s:%s' % server_address

sock.sendall(chr(robotPair))
data = sock.recv(1)
if ord(data[0]) != robotPairSucceeded:
    print 'pair request denied by server'
    sock.sendall(chr(kill))
    exit()

print 'paired with server'

while True:
    try:
        data = sock.recv(1)
        if ord(data[0]) == goto:
            data = sock.recv(4)
            while len(data) < 4:
                data += sock.recv(4 - len(data))
            titleLen = (ord(data[0]) << 24) | (ord(data[1]) << 16) | (ord(data[2]) << 8) | ord(data[3])
            data = sock.recv(titleLen)
            while len(data) < titleLen:
                data += sock.recv(titleLen - len(data))

            print 'nav request to', data
            thread = Thread(target = handleNav, args = (sock, data, ))
            thread.start()
        elif ord(data[0]) == cancelGoto:
            print 'got cancel request'
            cancel()
        elif ord(data[0]) == findMe:
            print 'got find me'
            thread = Thread(target = handleFind, args = (sock, ))
            thread.start()
        elif ord(data[0]) == cancelFindMe:
            print 'got cancel find'
            cancelFind()
        elif ord(data[0]) == ping:
            print 'pinged by server'
            sock.sendall(chr(ping))
        elif ord(data[0]) == kill:
            print 'got kill'
            break
        elif ord(data[0]) == goHomeReq:
            print 'got go home'
            thread = Thread(target = handleNav, args = (sock, None, ))
            thread.start()
        else:
            print 'got bad data'
            break
    except:
        print 'exception: ', sys.exc_info()[0]
        break

print 'shutting down...'
cancel()
try:
    sock.sendall(chr(kill))
except:
    pass
