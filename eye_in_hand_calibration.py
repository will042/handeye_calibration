import cv2
import glob
import numpy as np
import pathlib
import re
import math
from camera_calibration import calibrate_camera


#%% Tool Flange Pose Transformations

poses = np.loadtxt("poses.csv", delimiter=",", dtype=np.float64)

R_gripper2base = []
t_gripper2base = []

for pose in poses:

    t = (np.array(pose[0:3]) * 1000).reshape(3, 1)  # Translation vector (in mm)
    R = cv2.Rodrigues(pose[3:])[0]  # Rotation matrix from Rodrigues vector
    R_gripper2base.append(R)
    t_gripper2base.append(t)


#%% Default Camera Parameters
distortionCoefficients = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0])

fx = 1076.38
fy = 1076.69
cx = 1099.52
cy = 618.047

cameraMatrix = np.zeros((3, 3))  # creating camera matrix
cameraMatrix[0, 0] = fx
cameraMatrix[0, 2] = cx
cameraMatrix[1, 1] = fy
cameraMatrix[1, 2] = cy
cameraMatrix[2, 2] = 1

#%%
calibrate = False

if calibrate:
    distortionCoefficients, cameraMatrix = calibrate_camera()


#%% Load Images
directory = pathlib.Path(__file__).parent.resolve()  # get current file directory

image_paths = str(directory) + "/*.png"  # get image paths

filenames = [img for img in glob.glob(image_paths)]
filenames.sort(key=lambda var: [int(x) if x.isdigit() else x for x in
                                re.findall(r'[^0-9]|[0-9]+', var)])  # auto sort function for file name of images
images = []
for img in filenames:
    n = cv2.imread(img)
    images.append(n)

#%%

# calibration board
pattern_size = [20, 11]  # [height,width]
cell_size = 20  # in mm
image_shape = (1920, 1080)  # image size


# generate object points
objectPoints = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float64)
objectPoints[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
objectPoints = objectPoints * cell_size


#%% target to cam transformation
R_target2cam = []
t_target2cam = []
for i in range(0, len(images)):
    im = images[i]

    status, corners = cv2.findChessboardCorners(im, tuple(pattern_size))
    if status:
        cv2.drawChessboardCorners(im, tuple(pattern_size), corners, status)
        cv2.imwrite(str(directory) + "/images_detected/" + str(i) + ".png", im)
        status, rvecs, tvecs = cv2.solvePnP(objectPoints, corners, cameraMatrix,
                                            distortionCoefficients)  # estimate pose of target w.r.t camera
        R = cv2.Rodrigues(rvecs)[0]  # Rotation matrix from target w.r.t camera
        R_target2cam.append(R)
        t_target2cam.append(tvecs)
    else:
        print(i)


#%% Calculation
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
print(t_cam2gripper)
