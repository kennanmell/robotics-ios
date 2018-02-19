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
robotPair = 12
robotPairSucceeded = 13
robotPairFailed = 14
ping = 19

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
print 'sent data'
data = sock.recv(1)
print 'got data'
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

            if data == 'xy':
                print 'nav failed'
                sock.sendall(chr(gotoFailed))
            else:
                time.sleep(10)
                print 'nav succeeded'
                sock.sendall(chr(gotoDone))
        elif ord(data[0]) == ping:
            print 'pinged by server'
            sock.sendall(chr(ping))
        elif ord(data[0]) == kill:
            print 'got kill'
            break
        else:
            print 'got bad data'
            break
    except:
        print 'exception'
        break

print 'shutting down...'
try:
    sock.sendall(chr(kill))
except:
    pass
