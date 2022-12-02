#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from rtde_control import RTDEControlInterface as RTDEControl
import sys
import rtde
import rtde_config
import numpy as np
from time import sleep
import time
from rtde_receive import RTDEReceiveInterface as RTDEReceive

sys.path.append('..')


class controller(object):
    """
    Class to hold controllers.
    """

    # ===========================================================================
    def __init__(self):
        """
        Initializes UR Network, publishers, and subscribers.
        """
        self.force = np.array(np.zeros(6))

        # Inital Variables
        self.speed_z = 0
        self.force_d = -4  # Force to apply to specimen.
        self.force_u = 1.5

        # UR Setup
        self.rtde_c = RTDEControl("192.168.1.2")
        self.ROBOT_HOST = '192.168.1.2'
        self.ROBOT_PORT = 30004
        self.config_filename = '/home/icore/Workspaces/cv_pp_ws/src/ur5e_controller/control_loop_configuration.xml'  # Edit this for more data to log

        self.task_frame = [0, 0, 0, 0, 0, 0]
        self.selection_vector = [0, 0, 1, 0, 0, 0]  # Force Mode selected in the Z direction
        self.wrench_down = [0, 0, self.force_d, 0, 0, 0]
        self.wrench_up = [0, 0, self.force_u, 0, 0, 0]
        self.limits = [.1, .1, .005, 1, 1, 1]

        # Data Loggers/Receivers

        self.conf = rtde_config.ConfigFile(self.config_filename)
        self.state_names, self.state_types = self.conf.get_recipe('state')
        self.setp_names, self.setp_types = self.conf.get_recipe('setp')
        self.watchdog_names, self.watchdog_types = self.conf.get_recipe('watchdog')

        self.con = rtde.RTDE(self.ROBOT_HOST, self.ROBOT_PORT)
        self.con.connect()

        self.con.get_controller_version()
        self.con.send_output_setup(self.state_names, self.state_types)
        # self.setp = self.con.send_input_setup(self.setp_names, self.setp_types)

        if not self.con.send_start():
            sys.exit()

        # self.watchdog = self.con.send_input_setup(self.watchdog_names, self.watchdog_types)
        # self.watchdog.input_int_register_0 = 0
        self.pose = np.array(np.zeros(6))
        self.rx = 1.201
        self.ry = 2.896
        self.rz = 0
        # self.z_high = 0.01
        # self.z_low = 0.000

        self.z_high = 0.010 + 0.015
        self.z_low = 0.000 + 0.015

    # ===========================================================================
    def get_Pose_Data(self):
        # sleep(1)

        state_new = self.con.receive()  # Get latest UR Odometery
        time.sleep(0.01)  # Try to not break it
        ee_pose = [state_new.actual_TCP_pose[0], state_new.actual_TCP_pose[1], state_new.actual_TCP_pose[2],
                   state_new.actual_TCP_pose[3], state_new.actual_TCP_pose[4], state_new.actual_TCP_pose[5]]
        print("Pose received", ee_pose)
        self.pose = np.vstack([self.pose, ee_pose])
        force_l = [state_new.actual_TCP_force[0], state_new.actual_TCP_force[1], state_new.actual_TCP_force[2],
                   state_new.actual_TCP_force[3], state_new.actual_TCP_force[4], state_new.actual_TCP_force[5]]
        self.force = np.vstack([self.force, force_l])
        print("***********************")
        print("Force", force_l[2])
        # self.con.send_start()
        # self.con.send(self.setp)
        # self.con.disconnect()
        # self.con.connect()

    def gohome(self):

        # self.rtde_c.moveL([0.1402, -0.635, 0.1002, self.rx, self.ry, self.rz], .05, .1)
        self.rtde_c.moveL([0.1402, -0.721, 0.1002, self.rx, self.ry, self.rz], .05, .1)

    def movefunction(self, waypoints_sweeping3D):
        self.rtde_c.zeroFtSensor()

        contour_count = 0
        plt.figure(4)
        print('Number of Contours Detected: ', len(waypoints_sweeping3D))

        for i, waypoints in enumerate(waypoints_sweeping3D):
            print('Contour ', i, ' has shape: ', np.shape(waypoints))

            # plt.plot(waypoints[:,0], waypoints[:,1])
        all_points = np.empty((1, 2))
        # for i in range(len(waypoints_sweeping3D)):
        #     all_points = np.concatenate((all_points, waypoints_sweeping3D[i]))
        #
        # all_points = all_points[1:]
        # plt.plot(all_points[:,0], all_points[:,1])
        # plt.show()

        for waypoints in waypoints_sweeping3D:
            print('Contour number: ', contour_count)
            print('Length of current waypoints: ', len(waypoints))

            z = self.z_low

            for i in range(len(waypoints)):

                x = waypoints[i, 0] / 1000
                y = waypoints[i, 1] / 1000
                print('Moving to: ', x, ', ', y, ', ', z, ' i = ', i)

                self.rtde_c.moveL([x, y, z, self.rx, self.ry, self.rz], .1, .2)
                self.rtde_c.forceMode(self.task_frame, self.selection_vector, self.wrench_down, 2, self.limits)
                self.get_Pose_Data()

                if i == 0:
                    sleep(3)

                if i == (len(waypoints) - 1):
                    sleep(1)
                    # self.rtde_c.forceMode(self.task_frame, self.selection_vector, self.wrench_down, 2, self.limits)
                    self.rtde_c.forceMode(self.task_frame, self.selection_vector, self.wrench_up, 2, self.limits)
                    sleep(1)
                    # self.rtde_c.moveL([x, y, z, self.rx, self.ry, self.rz], .1, .2)
                    self.rtde_c.moveL([x, y, self.z_high, self.rx, self.ry, self.rz], .1, .2)

                # sleep(1)
                # state_new = self.con.receive() #Get latest UR Odometery
                # time.sleep(0.01) #Try to not break it
                # ee_pose = [state_new.actual_TCP_pose[0],state_new.actual_TCP_pose[1],state_new.actual_TCP_pose[2],state_new.actual_TCP_pose[3],state_new.actual_TCP_pose[4],state_new.actual_TCP_pose[5]]
                # print("Pose received", ee_pose)
                # self.pose = np.vstack([self.pose, ee_pose])
                # sleep(3)
            # sleep(3)
            print("Went out of the contour")
            # self.get_Pose_Data()

            contour_count = contour_count + 1

        np.savetxt("test_5/all_points_end_effector.csv", self.pose, delimiter=",")
        print("Pose RECEIVED")
        print("***********************")

        self.rtde_c.forceModeStop()
        # self.gohome()
        self.rtde_c.stopScript()

    def getallpoints(self, waypoints_sweeping3D):
        """
        Use for generating list of waypoints. Outputs to csv file. Not separated by contour. Includes z distance.
        """

        all_points = np.empty((1, 3))

        for waypoints in waypoints_sweeping3D:

            z = self.z_low

            for i in range(len(waypoints)):

                x = waypoints[i, 0] / 1000
                y = waypoints[i, 1] / 1000

                if i < (len(waypoints) - 1):
                    all_points = np.vstack((all_points, np.array([x, y, self.z_low])))

                if i == (len(waypoints) - 1):
                    all_points = np.vstack((all_points, np.array([x, y, self.z_high])))

        all_points = all_points[1:]
        np.savetxt("test_5/all_points.csv", all_points, delimiter=",")

        # self.con.send(self.watchdog)





