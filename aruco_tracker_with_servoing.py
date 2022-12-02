import numpy as np
import cv2
from time import sleep

from rtde_connection import Controller
from aruco_tracker import get_aruco_pose
from aruco_tracker import close_zed


key = ''

ur = Controller()

ee_pose = ur.get_pose_data()
q0 = ur.rtde_c.getInverseKinematics(ee_pose)

# print(ee_pose)


while key != 113:  # for 'q' key

    r_rod_cam_to_aruco, t_cam_to_aruco, gray = get_aruco_pose()

    key = cv2.waitKey(5)
    cv2.namedWindow("ZED", cv2.WINDOW_NORMAL)
    cv2.imshow("ZED", gray)
    cv2.resizeWindow("ZED", 1280, 720)

    if not np.isnan(np.sum(t_cam_to_aruco)) and np.isreal(np.sum(t_cam_to_aruco)):

        ee_pose = ur.get_pose_data()

        t_base_to_cam = np.asarray(ee_pose[0:3])

        r_base_to_cam = cv2.Rodrigues(np.asarray(ee_pose[3:]))[0]
        r_cam_to_aruco = cv2.Rodrigues(r_rod_cam_to_aruco)[0]

        # Target position
        Tt = np.dot(t_cam_to_aruco/1000, np.transpose(r_base_to_cam)) + t_base_to_cam

        Tt[2] = Tt[2] + 0.6  # maintain z distance from aruco target

        # Flip aruco coordinates to align with camera
        r = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

        # Target rotation
        Rt = cv2.Rodrigues(np.dot(np.dot(r_base_to_cam,r_cam_to_aruco), r))[0]

        # Target pose
        target_pose = np.array([Tt[0], Tt[1], Tt[2], Rt[0][0], Rt[1][0], Rt[2][0]])

        print(target_pose)

if np.isreal(np.sum(target_pose)):

    if ur.rtde_c.isPoseWithinSafetyLimits(target_pose):

        # q = ur.rtde_c.getInverseKinematics(target_pose)

        # q_new = [q0[0], q0[1], q0[2], q0[3], q0[4], q[5]]
        #
        # ur.rtde_c.servoJ(q_new, 0.05, 0.05, 0.005, 0.1, 100.0)

        ur.rtde_c.moveL(target_pose, .25, .25)


ur.rtde_c.servoStop()
ur.rtde_c.stopScript()
cv2.destroyAllWindows()
close_zed()
