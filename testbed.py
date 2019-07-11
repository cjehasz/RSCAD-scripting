
import socket
import struct

# Get IP Address
hostname = socket.gethostname()
IPAddr = socket.gethostbyname(hostname)
print("Your Computer Name is: " + hostname)
print("Your Computer IP Address is: " + IPAddr)

# RTDS import/export
UDP_IP_ctr = '137.99.169.55'
UDP_IP_rtds = '10.16.158.23'
UDP_PORT = 12345
sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP
sock.bind((UDP_IP_ctr, UDP_PORT))

# sock.settimeout(30)
#
# print(sock)
#
# loops = 1


while True:
    # print("Running loop: ", loops)
    data, addr = sock.recvfrom(1024)

    F = struct.unpack('>f', data)  # Number of 'f' is number of variables being sent or received
    print("Data: ", F)
    # print("Address: ", addr)

    # print(F)
    # # y = struct.pack('>ffff', ma, mb, mc, tri)
    # # sock.sendto(y, (UDP_IP_rtds, UDP_PORT))
    #
    # loops = loops + 1
    #
    # if loops == 100:
    #     break

