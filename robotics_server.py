import socket
import sys
from urllib2 import urlopen
from threading import Thread

pairedInstance = None
shutdown = False
rosSocket = None
pairedSpeaker = None
pairedPressure = None
navPending = False

def findMeResponse(connection):
    global pairedInstance
    global rosSocket
    global navPending

    findMeSucceeded = 26
    findMeFailed = 27

    data = rosSocket.recv(1)
    if ord(data[0]) != findMeSucceeded and ord(data[0]) != findMeFailed:
        print 'find failed (ros node unresponsive)'
        print 'disconnecting from ros node'
        rosSocket = None
        connection.sendall(chr(findMeFailed))
    else:
        print 'find finished'
        try:
            connection.sendall(data)
        except:
            pass
    navPending = False

def robotResponse(connection):
    global pairedInstance
    global rosSocket
    global navPending

    gotoDone = 8
    gotoFailed = 9
    cancelGotoSucceeded = 21
    goHomeSucceeded = 23

    data = rosSocket.recv(1)
    if pairedInstance == 12:
        if ord(data[0]) == goHomeSucceeded:
            print 'go home succeeded'
        else:
            print 'go home failed'
        pairedInstance = None
        return

    if ord(data[0]) != gotoDone and ord(data[0]) != gotoFailed and ord(data[0]) != cancelGotoSucceeded:
        print 'goto failed (ros node unresponsive)'
        print 'disconnecting from ros node'
        rosSocket = None
        connection.sendall(chr(gotoFailed))
    else:
        print 'goto finished'
        try:
            connection.sendall(data)
        except:
            pass
    navPending = False

def socketEventLoop(connection):
    global pairedInstance
    global shutdown
    global rosSocket
    global pairedSpeaker
    global navPending

    pair = 1
    pairSucceeded = 2
    pairFailed = 3
    unpair = 4
    speak = 5
    speakFailed = 6
    goto = 7
    gotoDone = 8
    gotoFailed = 9
    update = 10
    kill = 11

    robotPair = 12
    robotPairSucceeded = 13
    robotPairFailed = 14
    speakerPair = 15
    speakerPairSucceeded = 16
    speakerPairFailed = 17
    speakerUnpair = 18
    ping = 19
    cancelGoto = 20
    cancelGotoSucceeded = 21
    goHome = 22
    goHomeSucceeded = 23
    speakerSpeak = 24
    findMe = 25
    findMeSucceeded = 26
    findMeFailed = 27

    pressurePair = 28
    pressurePairSucceeded = 29
    pressurePairFailed = 30
    pressureDataNone = 31
    pressureDataHolding = 32
    pressureDataHigh = 33

    attempts = 0

    # Receive commands and respond to them
    while True:
        try:
            if shutdown:
                break

            if pairedInstance == connection and navPending == False:
                attempts += 1
                if attempts >= 14:
                    print 'force unpairing with', str(connection)
                    print 'sending robot home'
                    connection.sendall(chr(unpair))
                    pairedInstance = 12
                    rosSocket.sendall(chr(goHome))
                    thread = Thread(target = robotResponse, args = (connection,))
                    thread.start()
                    attempts = 0

            data = connection.recv(1)
            #instanceId = (ord(data[0]) << 8) | ord(data[1])
            if ord(data[0]) == pair:
                while len(data) < 6:
                    data += connection.recv(6 - len(data))

                leftHandMode = data[1] == 1
                userHeight = (ord(data[2]) << 24) | (ord(data[3]) << 16) | (ord(data[4]) << 8) | ord(data[5])
                if pairedInstance == None or pairedInstance == connection:
                    print 'paired with', str(connection)
                    pairedInstance = connection
                    response = chr(pairSucceeded)
                    connection.sendall(response)
                    attempts = 0
                else:
                    print 'denied pair from', str(connection)
                    response = chr(pairFailed)
                    connection.sendall(response)
            elif ord(data[0]) == unpair:
                print 'unpaired with', str(connection)
                if pairedInstance == connection:
                    if rosSocket is None:
                        pairedInstance = None
                    else:
                        print 'sending robot home'
                        pairedInstance = 12
                        rosSocket.sendall(chr(goHome))
                        thread = Thread(target = robotResponse, args = (connection,))
                        thread.start()
                    attempts = 0
            elif ord(data[0]) == speak:
                print 'got speak from', str(connection)
                if pairedInstance != connection:
                    continue
                attempts = 0
                if pairedSpeaker is None:
                    connection.sendall(chr(speakFailed))
                else:
                    pairedSpeaker.sendall(chr(speakerSpeak))
            elif ord(data[0]) == goto:
                print 'got goto from', str(connection)
                data += connection.recv(4)
                while len(data) < 5:
                    data += connection.recv(5 - len(data))

                titleLen = (ord(data[1]) << 24) | (ord(data[2]) << 16) | (ord(data[3]) << 8) | ord(data[4])
                data += connection.recv(titleLen)
                while len(data) < titleLen + 5:
                    data += connection.recv(titleLen + 5 - len(data))

                if pairedInstance != connection:
                    print 'ignoring goto (unpaired)'
                    continue

                if rosSocket is None:
                    print 'goto failed (ros node not connected)'
                    connection.sendall(chr(gotoFailed))

                navPending = True
                attempts = 0
                rosSocket.sendall(data)
                thread = Thread(target = robotResponse, args = (connection, ))
                thread.start()
            elif ord(data[0]) == findMe:
                print 'got find me from', str(connection)
                navPending = True
                attempts = 0
                thread = Thread(target = findMeResponse, args = (connection, ))
                thread.start()
            elif ord(data[0]) == cancelFindMe:
                print 'got cancel find request'
                if pairedInstance != connection:
                    print 'ignoring cancel find (unpaired)'
                    continue

                if rosSocket is None:
                    print 'can\'t cancel find (ros node not connected)'

                rosSocket.sendall(chr(cancelFindMe))
            elif ord(data[0]) == kill:
                print 'got kill from', str(connection)
                break
            elif ord(data[0]) == robotPair:
                if rosSocket == None or rosSocket == connection:
                    print 'set ros socket to', str(connection)
                    rosSocket = connection
                    connection.sendall(chr(robotPairSucceeded))
                    connection.settimeout(None)
	            return
                else:
                    rosSocket.sendall(chr(ping))
                    data = rosSocket.recv(1)
                    print 'got response to ping'
                    if  ord(data[0]) != ping:
                        print 'set ros socket to', str(connection)
                        rosSocket = connection
                        connection.sendall(chr(robotPairSucceeded))
                        connection.settimeout(None)
                        return
                    else:
                        print 'denied ros socket', str(connection)
                        connection.sendall(chr(robotPairFailed))
            elif ord(data[0]) == speakerPair:
                if pairedSpeaker == None or pairedSpeaker == connection:
                    print 'set speaker to', str(connection)
                    pairedSpeaker = connection
                    connection.sendall(chr(speakerPairSucceeded))
                    attempts = 0
                else:
                    print 'denied speaker', str(connection)
                    connection.sendall(chr(speakerPairFailed))
            elif ord(data[0]) == speakerUnpair:
                print 'unpaired speaker at', str(connection)
                if pairedSpeaker == connection:
                    pairedSpeaker = None
                    attempts = 0
            elif ord(data[0]) == pressurePair:
                if pairedPressure != None:
                    print 'unpairing with pressure', pairedPressure

                print 'set pressure to', str(connection)
                pairedPressure = connection
                connection.sendll(chr(pressurePairSucceeded))
            elif ord(data[0]) == pressureDataNone or ord(data[0]) == pressureDataHigh or ord(data[0]) == pressureDataHolding:
                print 'got pressure data', connection
                if pairedPressure == connection and !(rosSocket is None):
                    rosSocket.sendall(data[0])
                else:
                    print 'could not forward pressure data'
            elif ord(data[0]) == cancelGoto:
                print 'got cancel request'
                if pairedInstance != connection:
                    print 'ignoring cancel (unpaired)'
                    continue

                if rosSocket is None:
                    print 'can\'t cancel (ros node not connected)'
                    connection.sendall(chr(gotoFailed))

                rosSocket.sendall(chr(cancelGoto))
            else:
                print 'force kill: unrecognized data from', str(connection)
		print 'data:', ord(data[0])
                break
        except:
            continue

    # Clean up the connection
    print 'closing', str(connection)
    if pairedInstance == connection:
        pairedInstance = None
    if pairedSpeaker == connection:
        pairedSpeaker = None
    if rosSocket == connection:
        rosSocket = None
    try:
        connection.sendall(chr(kill))
    except:
        pass
    connection.close()

if len(sys.argv) != 2:
    print "Usage: python", sys.argv[0], "<portno>"
    exit()

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = (urlopen('http://ip.42.pl/raw').read(), int(sys.argv[1]))
#server_address = ('localhost', int(sys.argv[1]))
try:
    sock.bind(server_address)
except:
    print 'unable to bind to port'
    exit()

print 'running server on %s:%s' % server_address

# Listen for incoming connections
sock.listen(1)

while not shutdown:
    # Accept connections
    try:
        connection, client_address = sock.accept()
        connection.settimeout(5)
        print 'accepted connection', client_address
        thread = Thread(target = socketEventLoop, args = (connection, ))
        thread.start()
    except:
        print 'shutting down...'
        shutdown = True
        sock.close()

if not rosSocket is None:
    print 'sending kill to ros socket', rosSocket
    try:
        rosSocket.sendall(chr(kill))
    except:
        pass
