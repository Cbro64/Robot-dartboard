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
lower1 = np.array([0, 100, 30])
upper1 = np.array([10, 255, 255])
lower2 = np.array([170, 100, 30])
upper2 = np.array([180, 255, 255])

markerCount = 0
y1 = 0
y2 = 0
x1 = 0
x2 = 0
xavg = 0

# loop over the frames of the video
while True:
    # grab the current frame
    frame = vs.read()
    # convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

    # make threshold mask
    mask = cv2.inRange(hsv, lower1, upper1) + cv2.inRange(hsv, lower2, upper2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # loop over the contours and find largest
    markersize = 0
    marker = 0
    for c in cnts:
        area = cv2.contourArea(c)
        if area > markersize:
            markersize = area
            marker = c
            
    # find center
    if markersize > 0:
        M = cv2.moments(marker)
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])
        cv2.circle(frame, (x, y), 5, (255, 0, 0), -1)
        cv2.drawContours(frame, marker, -1, (0, 0, 255), 2)
        
        if markerCount == 1:
            xavg = int((x1 + x)/2)
            cv2.line(frame, (xavg, y1), (xavg, y), (0, 0, 255), 1)
            cv2.line(frame, (0, y), (640, y), (255, 0, 0), 1)
    
    if markerCount == 1:
        cv2.circle(frame, (x1, y1), 5, (0, 255, 0), -1)
        cv2.line(frame, (0, y1), (640, y1), (0, 255, 0), 1)
        
    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    # cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == ord("q"):
        if markerCount == 0:
            y1 = y
            x1 = x
            markerCount = 1
        else:
            y2 = y
            x2 = x
            break
            
cv2.circle(frame, (x2, y2), 5, (0, 255, 0), -1)            
cv2.line(frame, (xavg, y1), (xavg, y2), (0, 255, 0), 1)
cv2.line(frame, (0, y1), (640, y1), (0, 255, 0), 1)
cv2.line(frame, (0, y2), (640, y2), (0, 255, 0), 1)
cv2.imshow('frame', frame)
cv2.waitKey(5) & 0xFF

if xavg != 0:
    save = input("Save (y/n) ? ")
    if save == 'y':
        with open('calibration_values.txt', 'w') as calib_file:
            calib_file.write(str(xavg) + "," + str(y1) + "," + str(y2) + '\n')

vs.stop()
cv2.destroyAllWindows()
