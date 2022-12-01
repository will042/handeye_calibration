import numpy as np
import cv2
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
init_params.camera_fps = 30  # Set fps at 30

aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
parameters = aruco.DetectorParameters_create()
dist, mtx = calibrate_camera()


# Open the camera
err = zed.open(init_params)
if err != sl.ERROR_CODE.SUCCESS:
    exit(1)

# Grab an image
runtime_parameters = sl.RuntimeParameters()
if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
    # A new image is available if grab() returns ERROR_CODE.SUCCESS

    image = sl.Mat()

    key = ''
    while key != 113:  # for 'q' key
        err = zed.grab(runtime_parameters)
        if err == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)

            gray = cv2.cvtColor(image.get_data(), cv2.COLOR_BGR2GRAY)
            key = cv2.waitKey(5)
            corners, ids, rejected_img_points = aruco.detectMarkers(gray,
                                                                    aruco_dict,
                                                                    parameters=parameters,
                                                                    cameraMatrix=mtx,
                                                                    distCoeff=dist)
            if np.all(ids is not None):  # If there are markers found by detector
                for i in range(0, len(ids)):  # Iterate in markers
                    # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                    rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, mtx, dist)
                    (rvec - tvec).any()  # get rid of that nasty numpy value array error
                    aruco.drawDetectedMarkers(gray, corners)  # Draw A square around the markers
                    aruco.drawAxis(gray, mtx, dist, rvec, tvec, 0.01)  # Draw Axis
                    print(rvec)
            cv2.imshow("ZED", gray)
        else:
            key = cv2.waitKey(5)
    cv2.destroyAllWindows()

# Use 5x5 dictionary to find markers

# Marker detection parameters# lists of ids and the corners beloning to each id




# Close the camera
zed.close()



# get_image()
