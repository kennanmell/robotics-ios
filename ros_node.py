import socket
import sys
from urllib2 import urlopen
from threading import Thread
import time

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
    print 'unable to bind to server'
    exit()

print 'connected to %s:%s' % server_address

while True:
    data = sock.recv(1)
    if ord(data[0]) == 7:
        data = sock.recv(4)
        while len(data) < 4:
            data += sock.recv(4 - len(data))

        titleLen = (ord(data[0]) << 24) | (ord(data[1]) << 16) | (ord(data[2]) << 8) | ord(data[3])

        data = sock.recv(titleLen)
        while len(data < titleLen):
            data += sock.recv(titleLen - len(data))

        time.sleep(2)

        sock.sendall(chr(gotoDone))
    else:
        print 'got bad data'
