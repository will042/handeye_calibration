import numpy as np
import cv2 as cv
import cv2.aruco as aruco

#%%
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)
parameters = aruco.DetectorParameters_create()

dist = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0])
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

mtx = cameraMatrix


#%%
cap = cv.VideoCapture(1)

# if not cap.isOpened():
#     print("Cannot open camera")
#     exit()
#
# key = ''
# while key != 113:  # for 'q' key
#
#     ret, frame = cap.read()
#
#     if not ret:
#         print("Can't receive frame (stream end?). Exiting ...")
#         break
#
#     gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
#     corners, ids, rejected_img_points = aruco.detectMarkers(gray,
#                                                             aruco_dict,
#                                                             parameters=parameters,
#                                                             cameraMatrix=mtx,
#                                                             distCoeff=dist)
#     if np.all(ids is not None):  # If there are markers found by detector
#         for i in range(0, len(ids)):  # Iterate in markers
#             # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
#             rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, mtx, dist)
#             (rvec - tvec).any()  # get rid of that nasty numpy value array error
#             aruco.drawDetectedMarkers(gray, corners)  # Draw A square around the markers
#             aruco.drawAxis(gray, mtx, dist, rvec, tvec, 0.01)  # Draw Axis
#             print(rvec)
#
#     cv.imshow("ZED", gray)
#
# # When everything done, release the capture
# cap.release()
# cv.destroyAllWindows()


def track(matrix_coefficients, distortion_coefficients):
    i = 0
    while True:
        ret, frame = cap.read()
        # operations on the frame come here
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # Change grayscale
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters
        # lists of ids and the corners beloning to each id
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=matrix_coefficients,
                                                                distCoeff=distortion_coefficients)
        if np.all(ids is not None):  # If there are markers found by detector
            for i in range(0, len(ids)):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                           distortion_coefficients)
                (rvec - tvec).any()  # get rid of that nasty numpy value array error
                aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis

                x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
                y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]

                x_centerPixel = x_sum * .25
                y_centerPixel = y_sum * .25


        # Display the resulting frame
        cv.imshow('frame', frame)
        # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
        key = cv.waitKey(3) & 0xFF
        if key == ord('q'):  # Quit
            break

    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()


track(mtx, dist)