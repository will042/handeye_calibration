import cv2

from aruco_tracker import get_aruco_pose
from aruco_tracker import close_zed


key = ''

while key != 113:  # for 'q' key

    r_rod_cam_to_aruco, t_cam_to_aruco, gray = get_aruco_pose()

    key = cv2.waitKey(5)
    cv2.namedWindow("ZED", cv2.WINDOW_NORMAL)
    cv2.imshow("ZED", gray)
    cv2.resizeWindow("ZED", 1280, 720)

cv2.destroyAllWindows()
close_zed()
