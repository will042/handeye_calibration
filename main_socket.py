import socket_connection
from time import sleep

# Initialize the socket object for communication with the camera

while True:
    ur5 = socket_connection.SocketConnection('192.168.0.1', 8000)
    print(ur5.get_tcp())
    ur5.conn.close()
    ur5.s.close()
