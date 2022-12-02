import numpy as np
import cv2.aruco as aruco
import pyzed.sl as sl
import cv2
from camera_calibration import calibrate_camera


# Create a ZED camera object
zed = sl.Camera()

# Set configuration parameters
init_params = sl.InitParameters()
init_params.camera_image_flip = sl.FLIP_MODE.OFF
init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use millimeter units (for depth measurements)
init_params.camera_fps = 30  # Set fps at 30

aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
parameters = aruco.DetectorParameters_create()
# dist, mtx = calibrate_camera()
dist = np.array([[1.63571239e-02, -8.80685747e-02, -5.17807723e-04, -2.95441702e-05, 9.14120653e-02]])
mtx = np.array([[1.05272765e+03, 0.00000000e+00, 9.54229979e+02],
                [0.00000000e+00, 1.05064323e+03, 5.39977731e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)

# Grab an image
runtime_parameters = sl.RuntimeParameters()


def get_aruco_pose():

    rvec = np.empty((1, 1, 3))
    point3d = np.empty((3,))

    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:

        image = sl.Mat()
        point_cloud = sl.Mat()

        err = zed.grab(runtime_parameters)
        if err == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            gray = cv2.cvtColor(image.get_data(), cv2.COLOR_BGR2GRAY)
            # key = cv2.waitKey(5)
            corners, ids, rejected_img_points = aruco.detectMarkers(gray,
                                                                    aruco_dict,
                                                                    parameters=parameters,
                                                                    cameraMatrix=mtx,
                                                                    distCoeff=dist)
            if np.all(ids is not None):  # If there are markers found by detector
                for i in range(0, len(ids)):  # Iterate in markers
                    # Estimate pose of each marker and return the values rvec and tvec
                    rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, mtx, dist)
                    (rvec - tvec).any()  # get rid of that nasty numpy value array error
                    aruco.drawDetectedMarkers(gray, corners)  # Draw A square around the markers
                    aruco.drawAxis(gray, mtx, dist, rvec, tvec, 0.01)  # Draw Axis


                    x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
                    y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]
                    x_centerPixel = x_sum * .25
                    y_centerPixel = y_sum * .25

                    point3d = point_cloud.get_value(x_centerPixel, y_centerPixel)[1][0:3]

    return rvec, point3d, gray


def close_zed():
    zed.close()