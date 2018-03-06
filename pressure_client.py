kill = 11
ping = 19
pressurePair = 28
pressurePairSucceeded = 29
pressurePairFailed = 30
pressureDataNone = 31
pressureDataHolding = 32
pressureDataHigh = 33

if len(sys.argv) != 3:
    print "Usage: python", sys.argv[0], "<server-ip> <server-portno>"
    exit()

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = (sys.argv[1], int(sys.argv[2]))
try:
    sock.connect(server_address)
except:
    print 'unable to connect to server'
    exit()

print 'connected to server %s:%s' % server_address

sock.sendall(chr(pressurePair))
data = sock.recv(1)
if ord(data[0]) != pressurePairSucceeded:
    print 'pair request denied by server'
    sock.sendall(chr(kill))
    exit()

print 'paired with server'

# TODO: read pressure and use sock.sendall(chr(pressureData[None, Holding, High])) to send changes
