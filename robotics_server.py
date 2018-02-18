import socket
import sys
from urllib2 import urlopen
from threading import Thread

shutdown = False
rosSocket = None

def socketEventLoop(connection):
    global shutdown
    global rosSocket

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

    pairedInstance = None
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
                    pairedInstance = connection #instanceId
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
                    break
                attempts = 0
                rosSocket.sendall(chr(speak))
            elif ord(data[0]) == goto:
                print 'got goto from', str(connection)
                if pairedInstance != connection:
                    break

                data += connection.recv(4)
                while len(data) < 5:
                    data += connection.recv(5 - len(data))

                titleLen = (ord(data[0]) << 24) | (ord(data[1]) << 16) | (ord(data[2]) << 8) | ord(data[3])

                data += connection.recv(titleLen)
                while len(data < titleLen + 5):
                    data += connection.recv(titleLen + 5 - len(data))

                rosSocket.sendall(data)
                data = rosSocket.recv(1)
                connection.sendall(data)
                #locLength = (ord(data[1]) << 24) | (ord(data[2]) << 16) | (ord(data[3]) << 8) | ord(data[4]])
                attempts = 0

                if goto():
                    connection.sendall(chr(gotoDone))
                else:
                    connection.sendall(chr(gotoFailed))
            elif ord(data[0]) == kill:
                print 'got kill from', str(connection)
                if pairedInstance == connection:
                    pairedInstance = None
                    attempts = 0
                break
            else:
                print 'unrecognized data from', str(connection)
                break
        except:
            continue

    # Clean up the connection
    print 'closing', str(connection)
    if pairedInstance == connection:
        pairedInstance = None
        attempts = 0
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
#server_address = (urlopen('http://ip.42.pl/raw').read(), int(sys.argv[1]))
server_address = ('localhost', int(sys.argv[1]))
print >>sys.stderr, 'starting up on %s port %s' % server_address
try:
    sock.bind(server_address)
except:
    print 'unable to bind to port'
    exit()

# Listen for incoming connections
sock.listen(1)

rosConnection, ros_address = sock.accept()
rosSocket = rosConnection
print 'accepted ros connection: ', ros_address

while not shutdown:
    # Accept connections
    try:
        connection, client_address = sock.accept()
        connection.settimeout(5)
        thread = Thread(target = socketEventLoop, args = (connection, ))
        thread.start()
        print 'accepted connection: ', client_address
    except:
        print 'shutting down...'
        shutdown = True
        sock.close()
