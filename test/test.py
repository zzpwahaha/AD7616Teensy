import socket
import sys
from time import sleep

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect the socket to the port where the server is listening
server_address = ('10.10.0.10', 80)
print(sys.stderr, 'connecting to %s port %s' % server_address)
sock.connect(server_address)

try:
    # Send data
    # message = b'This is the message.  It will be repeated.\n'
    # print(sys.stderr, 'sending "%s"' % message)
    # sock.sendall(message)

    # # Look for the response
    # amount_received = 0
    # amount_expected = len(message)
    
    message = b'(rng,\xFD\xFF\xFF\xFF)#'
    print(sys.stderr, 'sending "%s"' % message)
    sock.sendall(message)

    # while amount_received < amount_expected:
    data = sock.recv(4095)
    # amount_received += len(data)
    print(sys.stderr, 'received "%s"' % data)

    # sleep(0.100)

    message = b'(*\x01\x01,100)#'
    print(sys.stderr, 'sending "%s"' % message)
    sock.sendall(message)

    data = sock.recv(4095)
    # amount_received += len(data)
    print(sys.stderr, 'received "%s"' % data)


    message = b'(trg, )#'
    print(sys.stderr, 'sending "%s"' % message)
    sock.sendall(message)
    
    data = sock.recv(4095)
    # amount_received += len(data)
    print(sys.stderr, 'received "%s"' % data)

finally:
    print(sys.stderr, 'closing socket')
    sock.close()