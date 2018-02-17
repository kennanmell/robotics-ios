import socket
import sys
from urllib2 import urlopen
from threading import Thread

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

serverSocket = None

def clientEventLoop(connection):
    global serverSocket
    while true:
        data = connection.recv(1)
        serverSocket.sendall(data)

def serverEventLoop(connection):
    global serverSocket
    while true:
        data = serverSocket.recv(1)
        connection.sendall(data)

while True:
    # Accept connections
    try:
        connection, client_address = sock.accept()
        if serverSocket == None:
            serverSocket = connection
            thread = Thread(target = serverEventLoop, args = (connection, ))
            thread.start()
        else:
            thread = Thread(target = clientEventLoop, args = (connection, ))
            thread.start()
            break
        print 'accepted connection: ', client_address
    except:
        print 'shutting down...'
        shutdown = True
        sock.close()
