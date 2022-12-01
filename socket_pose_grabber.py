from socket_connection import SocketConnection


def get_pose():
    float_pose = []
    while len(float_pose) == 0:
        ur5 = SocketConnection('192.168.0.1', 8000)
        tcp = str(ur5.get_tcp())
        tcp = tcp[4:-2]
        float_pose = [float(value) for value in tcp.split(',')]
        print('Current pose:')
        print(float_pose)
        ur5.conn.close()
        ur5.s.close()

    return float_pose


