# Cameron Brown 2019
# Monash University FYP S1
# This code allows our raspberry pi cameras to detect where they are in relation to
# the dart board plane of motion, so it can send correct target position signals.
# Relies on having two red markers at each end of the motion axis.

from pivideostream import PiVideoStream
import time
import imutils
import cv2
import numpy as np

vs = PiVideoStream().start()
time.sleep(1) # allow camera to warm up

# define range of red color in HSV
lower1 = np.array([0, 100, 10])
upper1 = np.array([5, 255, 255])
lower2 = np.array([170, 100, 10])
upper2 = np.array([180, 255, 255])

# loop over the frames of the video
while True:
    # grab the current frame
    frame = vs.read()
    # convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    # make threshold mask
    mask = cv2.inRange(hsv, lower1, upper1) + cv2.inRange(hsv, lower2, upper2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # loop over the contours and find 2 largest
    marker1size = 0
    marker2size = 0
    marker1 = 0
    marker2 = 0
    for c in cnts:
        # if the contour is too small, ignore it
        area = cv2.contourArea(c)
        if area > marker1size:
            marker2 = marker1
            marker2size = marker1size
            marker1 = c
            marker1size = area
        elif area > marker2size:
            marker2 = c
            marker2size = area

    # find centers of the two contours
    if marker1size > 0 and marker2size > 0:
        M1 = cv2.moments(marker1)
        M2 = cv2.moments(marker2)
        x1 = int(M1["m10"] / M1["m00"])
        y1 = int(M1["m01"] / M1["m00"])
        x2 = int(M2["m10"] / M2["m00"])
        y2 = int(M2["m01"] / M2["m00"])
        cv2.circle(frame, (x1, y1), 5, (0, 255, 0), -1)
        cv2.circle(frame, (x2, y2), 5, (0, 255, 0), -1)
        cv2.drawContours(frame, [marker1, marker2], -1, (0, 0, 255), 2)

    	# draw lines on frame to show range of motion
        xavg = int((x1 + x2)/2)
        cv2.line(frame, (xavg, y1), (xavg, y2), (0, 255, 0), 1)
        cv2.line(frame, (0, y1), (640, y1), (0, 255, 0), 1)
        cv2.line(frame, (0, y2), (640, y2), (0, 255, 0), 1)

    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    # cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == ord("q"):
        break

if xavg is not None:
    save = input("Save (y/n) ? ")
    k = cv2.waitKey(10000) & 0xFF
    if save == 'y':
        with open('calibration_values.txt', 'w') as calib_file:
            calib_file.write(str(xavg) + "," + str(y1) + "," + str(y2) + '\n')

vs.stop()
cv2.destroyAllWindows()
