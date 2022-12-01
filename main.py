from socket_pose_grabber import get_pose
from capture_image import get_image

import numpy as np
import cv2

image = get_image()

pose = get_pose()

t = np.asarray(pose[0:3])
R = np.asarray(pose[3:6])

Rmat, *_ = cv2.Rodrigues(R) # ee to base

Rmat2 = np.linalg.inv(Rmat) # base to ee

