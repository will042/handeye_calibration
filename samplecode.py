import enum
from turtle import degrees
from venv import create
import cv2
import numpy as np
import glob
import math
# import matplotlib.pyplot as plt
import csv
import copy
import re
# import pandas as pd
import pathlib
# from scipy.spatial.transform import Rotation as R


def create_matrix(pose):
    # get info from base 2 gripper
    translation = np.array(pose[0:3]) * 1000  # Translation vector (in mm)
    rotation = np.array(pose[3:6])  # Rotation vector unscaled (in rad)
    rv_len = math.sqrt(
        math.pow(rotation[0], 2) + math.pow(rotation[1], 2) + math.pow(rotation[2], 2))  # Orthogonal Length (in rad)
    scale = 1 - 2 * math.pi / rv_len  # calculate scaling factor
    rotation_scaled = scale * rotation  # Rotation vector (in rad)
    t_Mtx = np.array([[translation[0]], [translation[1]], [translation[2]]]).reshape(3,
                                                                                     1)  # Column vector of translation vector (in mm)
    R_Mtx_rodri = cv2.Rodrigues(rotation_scaled)[
        0]  # Rodrigues operator for calculting rotation matrix --> scipy converter delivers the same

    return R_Mtx_rodri, t_Mtx


def matrices_from_txt(path):
    # parse information from base 2 gripper from log file
    df = pd.read_csv(path, header=None)  # define pandas dataframe
    df.to_numpy()  # convert pandas to numpy
    RMtx = []
    tMtx = []
    # print(df)
    for i in range(len(df[0])):
        pose = [df[6][i], df[7][i], df[8][i], df[9][i], df[10][i],
                df[11][i]]  # 6,7,8 : X;Y,Z (m)| 9,10,11: unscaled RX,RY,RZ (rad)
        print(pose, 'pose')
        R, t = create_matrix(pose)
        print(R, 'RRobo')
        print(t, 'trobo')
        RMtx.append(R)
        tMtx.append(t)

    return RMtx, tMtx


if __name__ == "__main__":
    directory = pathlib.Path(__file__).parent.resolve()  # get current file directory
    robot_paths = str(directory) + '/images/coordinate.csv'  # locate log file directory
    image_paths = str(directory) + "/images/*.png"  # get image paths

    # coefficients
    distortionCoefficients = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0])  # distortion Coefficients (D415 default)

    # camera matrix, define here or calibrate later
    calibrate = False  # True to autocalibrate, False = D415 1280x720 default setup
    f_x = 930.172  # (D415 default)
    f_y = 930.172
    c_x = 627.637
    c_y = 358.512

    cameraMatrix = np.zeros((3, 3))  # creating camera matrix
    cameraMatrix[0, 0] = f_x
    cameraMatrix[0, 1] = 0
    cameraMatrix[0, 2] = c_x
    cameraMatrix[1, 0] = 0
    cameraMatrix[1, 1] = f_y
    cameraMatrix[1, 2] = c_y
    cameraMatrix[2, 0] = 0
    cameraMatrix[2, 1] = 0
    cameraMatrix[2, 2] = 1

    # calibration board
    patternsize = [6, 4]  # [height,width]
    cell_size = 35  # in mm
    image_shape = (1280, 720)  # image size

    # generate object points
    objectPoints = np.zeros((patternsize[0] * patternsize[1], 3), np.float32)
    objectPoints[:, :2] = np.mgrid[0:patternsize[0], 0:patternsize[1]].T.reshape(-1, 2)
    objectPoints = objectPoints * cell_size

    # load images
    filenames = [img for img in glob.glob(image_paths)]
    filenames.sort(key=lambda var: [int(x) if x.isdigit() else x for x in
                                    re.findall(r'[^0-9]|[0-9]+', var)])  # auto sort function for file name of images
    images = []
    for img in filenames:
        n = cv2.imread(img)
        images.append(n)

    # load rotation and translation from text file
    R_gripper2base, t_gripper2base = matrices_from_txt(robot_paths)

    if calibrate:
        # camera calibration
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # termination criteria
        corners_calib = []  # image coordinates of corners
        objp = []  # N times object coordinates
        for i in range(0, len(images)):
            im = images[i]
            im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            status, corners = cv2.findChessboardCorners(im, tuple(patternsize))

            corners2 = cv2.cornerSubPix(im, corners, (11, 11), (-1, -1), criteria)
            corners_calib.append(corners2)
            objp.append(objectPoints)
            # print(len(objectPoints))

        corners_calib = np.squeeze(np.asarray(corners_calib))

        objp = np.asarray(objp)
        print("calibrating camera...")
        ret, cameraMatrix, distortionCoefficients, rvecs, tvecs = cv2.calibrateCamera(objp, corners_calib, image_shape,
                                                                                      None, None)

        mean_error = 0
        for i in range(0, len(objp)):
            imgpoints2, _ = cv2.projectPoints(objp[i], rvecs[i], tvecs[i], cameraMatrix, distortionCoefficients)
            error = cv2.norm(corners_calib[i], np.squeeze(imgpoints2), cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print("total error: {}".format(mean_error / len(objp)))
    print("camera matrix: ")
    print(cameraMatrix)
    print("distortion coefficients:")
    print(distortionCoefficients)

    # target to cam transformation
    R_target2cam = []
    t_target2cam = []
    for i in range(0, len(images)):
        im = images[i]

        status, corners = cv2.findChessboardCorners(im, tuple(patternsize))
        if status:
            cv2.drawChessboardCorners(im, tuple(patternsize), corners, status)
            cv2.imwrite(str(directory) + "/images_detected/images" + str(i) + ".png", im)
            status, rvecs, tvecs = cv2.solvePnP(objectPoints, corners, cameraMatrix,
                                                distortionCoefficients)  # estimate pose of target w.r.t camera
            R = cv2.Rodrigues(rvecs)[0]  # Rotation matrix from target w.r.t camera
            R_target2cam.append(R)
            t_target2cam.append(tvecs)
        else:
            print(i)

    # Calculation
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
                                                        method=cv2.CALIB_HAND_EYE_PARK)
    print("PARK")

    print(R_cam2gripper)
    print(t_cam2gripper)

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
                                                        method=cv2.CALIB_HAND_EYE_ANDREFF)
    print("ANDREFF")
    print(R_cam2gripper)
    print(t_cam2gripper)

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
                                                        method=cv2.CALIB_HAND_EYE_DANIILIDIS)

    print("DANIILIDIS")
    print(R_cam2gripper)
    print(t_cam2gripper)

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
                                                        method=cv2.CALIB_HAND_EYE_HORAUD)

    print("HORAUD")
    print(R_cam2gripper)
    print(t_cam2gripper)

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam,
                                                        method=cv2.CALIB_HAND_EYE_TSAI)

    print("TSAI")
    print(R_cam2gripper)