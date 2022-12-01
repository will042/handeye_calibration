from socket_pose_grabber import get_pose
from capture_image import get_image

import csv

'''
Used to capture calibration images and poses for eye in hand calibration
'''

while True:
    rowcount = 1
    for row in open('poses.csv'):
        rowcount += 1

    image = get_image().write('img' + "{:02d}".format(rowcount) + '.png')  # press 'q' to acquire

    pose = get_pose()

    file = open('poses.csv', 'a+', newline='')

    with file:
        write = csv.writer(file)
        write.writerow(pose)