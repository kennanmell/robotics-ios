import socket
import sys
from urllib2 import urlopen
from threading import Thread

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
shutdown = False

def goto(location):
    # TODO: Implement this.
    # Should navigate the robot to the location specified by <location>.bag
    # Return True if navigation succeeds, False otherwise
    # Should not return until navigation completes
    return False

def speak():
    # TODO: Implement this.
    # Should cause the robot to say something like "Assisance available here."
    # Should return immediately.
    pass

def socketEventLoop(connection):
    global pairedInstance
    global shutdown

    # Receive the data in small chunks and retransmit it
    attempts = 0
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
                speak()
            elif ord(data[0]) == goto:
                print 'got goto from', str(connection)
                if pairedInstance != connection:
                    break
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
server_address = (urlopen('http://ip.42.pl/raw').read(), int(sys.argv[1]))
#server_address = ('localhost', int(sys.argv[1]))
print >>sys.stderr, 'starting up on %s port %s' % server_address
try:
    sock.bind(server_address)
except:
    print 'unable to bind to port'
    exit()

# Listen for incoming connections
sock.listen(1)

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
