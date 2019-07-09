
import socket
import struct

# Python Program to Get IP Address
hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)
print("Your Computer Name is: " + hostname)
print("Your Computer IP Address is: " + IPAddr)

# RTDS import/export
UDP_IP_ctr = IPAddr
UDP_IP_rtds = '10.16.158.36'
UDP_PORT = 10101
sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP
sock.bind((UDP_IP_ctr, UDP_PORT))

print("Listening on port:", UDP_PORT)
print(sock)

loops = 1


while True:
    print("Running loop: ", loops)
    # data, addr = sock.recvfrom(4096)
    data = sock.recv(1024)
    print("Data received")
    F = struct.unpack('>fffffffff', data)  # Number of 'f' is number of variables, both import and export
    print("Data: ", data)
    print("Address: ", addr)

    print(F)
    # y = struct.pack('>ffff', ma, mb, mc, tri)
    # sock.sendto(y, (UDP_IP_rtds, UDP_PORT))

    loops = loops + 1

    if loops == 100:
        break




