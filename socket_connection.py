"""
Used for communicating with the Universal Robot
"""
import socket
from time import sleep


class SocketConnection:
    """
    Creates a new ``socket_connection`` instance, which represents a connection to the UR5e or the Cognex Camera.
    `host` and `port` should be the IP address and desired socket port for the computer;
    """

    def __init__(self, host, port):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((host, port))  # Bind to the port
        print("Connection bound to: ", host, ":", str(port))

    def send_ur5_msg(self, msg):
        """
        Used to send a message to the UR5e
        """
        self.s.listen(5)  # Now wait for client connection.
        print("Listening for UR5e message")
        self.conn, self.addr = self.s.accept()  # Establish connection with client.
        self.TCP = self.conn.recv(1024)  # Recieve information (Tool Center Point)
        print("Current UR5e Tool Center Point: ", self.TCP)
        sleep(1)
        print("Sending: ", msg)
        self.conn.send(msg)  # Send encoded message
        print("Message Sent")

    def get_tcp(self):
        """
        Used to get current TCP
        """
        self.s.listen(5)  # Now wait for client connection.
        print("Listening for UR5e message")
        self.conn, self.addr = self.s.accept()  # Establish connection with client.
        self.TCP = self.conn.recv(1024)  # Recieve information (Tool Center Point)
        print("Current UR5e Tool Center Point: ", self.TCP)

        return self.TCP


