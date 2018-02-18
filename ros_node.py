import socket
import sys
from urllib2 import urlopen
from threading import Thread

if len(sys.argv) != 3:
    print "Usage: python", sys.argv[0], "<server-ip> <server-portno>"
    exit()

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = (sys.argv[1], int(sys.argv[2]))
#server_address = ('localhost', int(sys.argv[1]))
print >>sys.stderr, 'connecting to %s port %s' % server_address
try:
    sock.connect(server_address)
except:
    print 'unable to bind to server'
    exit()

while True:
    data = sock.recv(1)
    if ord(data[0]) == 5:
        print 'got data 5 -> speak'
    else:
        print 'got bad data'
