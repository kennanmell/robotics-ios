import socket
import sys
from urllib2 import urlopen
from threading import Thread

pairedInstance = None
shutdown = False
rosSocket = None
pairedSpeaker = None

def socketEventLoop(connection):
    global pairedInstance
    global shutdown
    global rosSocket
    global pairedSpeaker

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
    speakerPair = 15
    speakerPairSucceeded = 16
    speakerPairFailed = 17
    speakerUnpair = 18
    ping = 19

    attempts = 0

    # Receive commands and respond to them
    while True:
        try:
            if shutdown:
                break

            if pairedInstance == connection:
                attempts += 1
                if attempts >= 14:
                    print 'force unpairing with', str(connection)
                    connection.sendall(chr(unpair))
                    pairedInstance = None
                    attempts = 0

            data = connection.recv(1)
            #instanceId = (ord(data[0]) << 8) | ord(data[1])
            if ord(data[0]) == pair:
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
                    pairedInstance = None
                    attempts = 0
            elif ord(data[0]) == speak:
                print 'got speak from', str(connection)
                if pairedInstance != connection:
                    continue
                attempts = 0
                rosSocket.sendall(chr(speak))
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

                rosSocket.sendall(data)
                data = rosSocket.recv(1)
                if ord(data[0]) != gotoDone and ord(data[0]) != gotoFailed:
                    print 'goto failed (ros node unresponsive)'
                    print 'disconnecting from ros node'
                    rosSocket = None
                    connection.sendall(chr(gotoFailed))
                else:
                    print 'goto finished'
                    connection.sendall(data)
                #locLength = (ord(data[1]) << 24) | (ord(data[2]) << 16) | (ord(data[3]) << 8) | ord(data[4]])
                attempts = 0
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
                    connection.settimeout(None)
                    attempts = 0
                else:
                    print 'denied speaker', str(connection)
                    connection.sendall(chr(speakerPairFailed))
            elif ord(data[0]) == speakerUnpair:
                if pairedSpeaker == connection:
                    pairedSpeaker = None
                    attempts = 0
                    connection.settimeout(5)
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
