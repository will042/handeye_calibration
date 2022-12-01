from rtde_control import RTDEControlInterface as RTDEControl
import rtde
from rtde import rtde_config
import time
import sys
import cv2
import numpy as np


class Controller(object):
    """
    Class to hold controllers.
    """

    def __init__(self):
        """
        Initializes UR Network, publishers, and subscribers.
        """

        # UR Setup
        self.rtde_c = RTDEControl("192.168.0.2")
        self.ROBOT_HOST = '192.168.0.2'
        self.ROBOT_PORT = 30004

        # Data Loggers/Receivers

        self.config_filename = 'control_loop_configuration.xml'
        self.conf = rtde_config.ConfigFile(self.config_filename)
        self.state_names, self.state_types = self.conf.get_recipe('state')
        self.setp_names, self.setp_types = self.conf.get_recipe('setp')
        self.watchdog_names, self.watchdog_types = self.conf.get_recipe('watchdog')

        self.con = rtde.RTDE(self.ROBOT_HOST, self.ROBOT_PORT)
        self.con.connect()

        self.con.get_controller_version()
        self.con.send_output_setup(self.state_names, self.state_types)

        if not self.con.send_start():
            sys.exit()

    def get_pose_data(self):
        """
        Returns current TCP pose: [x, y, z, rx, ry, rz]
        """

        state_new = self.con.receive()  # Get latest data
        time.sleep(0.01)  # Try to not break it
        ee_pose = [state_new.actual_TCP_pose[0], state_new.actual_TCP_pose[1], state_new.actual_TCP_pose[2],
                   state_new.actual_TCP_pose[3], state_new.actual_TCP_pose[4], state_new.actual_TCP_pose[5]]

        return ee_pose


#%%
con = Controller()


#%%
pose = con.get_pose_data()

