import socket
import sys
from time import sleep, time_ns
from time import time

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('10.10.0.10', 80)
print(sys.stderr, 'connecting to %s port %s' % server_address)
sock.connect(server_address)

try:
    t0 = time_ns()

    # Send data    
    message = b'(rng,\xFD\xFF\xFF\xFF)#'
    print(sys.stderr, 'sending "%s" at time %d us' % (message, (time_ns()-t0)/1000.0))
    sock.sendall(message)

    # receive data
    # NOTE !!!!! If there is not return, do not try to recv(), which will make the connection freeze
    data = sock.recv(4095)
    print(sys.stderr, 'received "%s" at time %d us' % (data, (time_ns()-t0)/1000.0))

    message = b'(*\x01\x01,100)#'
    print(sys.stderr, 'sending "%s" at time %d us' % (message, (time_ns()-t0)/1000.0))
    sock.sendall(message)

    # data = sock.recv(4095)
    # print(sys.stderr, 'received "%s"' % data)

    message = b'(trg, )#'
    print(sys.stderr, 'sending "%s" at time %d us' % (message, (time_ns()-t0)/1000.0))
    sock.sendall(message)
    
    data = sock.recv(1<<16)
    print(sys.stderr, 'received "%s" at time %d us' % (data, (time_ns()-t0)/1000.0))

finally:
    print(sys.stderr, 'closing socket')
    sock.close()