# Robotic dartboard raspberry pi code for final year project
# Monash University
# Cameron Brown 2018
#
# Frame capture method adapted from
# https://gist.github.com/CarlosGS/b8462a8a1cb69f55d8356cbb0f3a4d63
# @CarlosGS May 2017

from pivideostream import PiVideoStream
import cv2
import numpy as np
import subprocess as sp
import time
import atexit
import serial
import struct
import imutils


# Open serial port
psoc = serial.Serial('/dev/ttyS0', 57600)

# Video capture parameters
(w, h) = (640, 480)
bytesPerFrame = w * h
fps = 35  # setting to 250 will request the maximum framerate possible
# image processing will slow down the pipeline, so the requested FPS should be set just below the pipeline speed

# Projection parameters
f = 540 # recalculate this - focal length in pixels
u0 = round(w/2)  # pixel coordinates of principal point: width / 2
v0 = round(h/2)  # height / 2

# pixel to camera frame transformation matrix
# M = [f 0 u0; 0 f v0; 0 0 1]
# only need inverse of M, pre-calculate for speed
M = np.array([[f, 0, u0], [0, f, v0], [0, 0, 1]])
invM = np.linalg.inv(M)

# rotations from camera to world(sidepi): Rx(180), Ry(-90)
# Rx(180) = [1 0 0; 0 -1 0; 0 0 -1]
# Ry(-90) = [0 0 -1; 0 -1 0; -1 0 0]
# R = RxRy = [0 0 -1; 0 -1 0; -1 0 0]
t = [0, -1.675, 1.2]  # translation in new frame orientation
T = np.array([[0, 0, -1, t[0]], [-1, 0, 0, t[1]], [0, 1, 0, t[2]]])
T = T.T # transpose, used in frame loop

#print(deltaC)

#  Start camera process
vs = PiVideoStream().start()
cv2.waitKey(1000)

# define range of red color in HSV
lower1 = np.array([36, 25, 25])
upper1 = np.array([86, 230, 230])
#lower2 = np.array([175, 100, 30])
#upper2 = np.array([180, 255, 255])

lastFrame = None
capture = False

xCutoff = 630  # ignore pixels past this value in horizontal direction
motionThreshold = 0.05 #% percentage of frame required for motion detection
motionThreshold = motionThreshold * w * h / 100 # convert to number of pixels
frameCount = 0
start_time = 0

# Capture loop
while True:
    frame = vs.read()
    # convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # make threshold mask
    mask = cv2.inRange(hsv, lower1, upper1) #+ cv2.inRange(hsv, lower2, upper2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # loop over the contours and find largest
    markerSize = 0
    marker = 0
    for c in cnts:
        # if the contour is too small, ignore it
        area = cv2.contourArea(c)
        if area > markerSize:
            marker = c
            markerSize = area
    
    # Exit if 'q' key is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
        
    # find center of the contour
    if markerSize > 0:
        M1 = cv2.moments(marker)
        x1 = int(M1["m10"] / M1["m00"])
        y1 = int(M1["m01"] / M1["m00"])
        cv2.circle(frame, (x1, y1), 5, (0, 255, 0), -1)
        #cv2.drawContours(frame, marker, -1, (0, 0, 255), 2)
    
        frameCount += 1
        
        # Convert from pixel coords to frame coords
        pix = np.array([x1, y1, 1])
        coords = np.dot(invM, pix)

        # Calculate direction vector of 3D line that explains point in image
        # as the cross product of two 3D planes
        xline = np.array([[-1], [0], [coords[0]]])
        yline = np.array([[0], [-1], [coords[1]]])
        xplane = np.dot(T, xline) # vector perpendicular to the plane
        yplane = np.dot(T, yline)
        # normalise and remove 4th element
        if xplane[3] != 0:
            xplane = np.divide(xplane[0:3], xplane[3])
        else:
            xplane = xplane[0:3]
        if yplane[3] != 0:
            yplane = np.divide(yplane[0:3], yplane[3])
        else:
            yplane = yplane[0:3]

        V = np.cross(xplane.T, yplane.T)
        V1 = V[0]

        # Send to Psoc
        print(V)
        psoc.write(V.astype('f').tostring()) # 12 bytes, 4 per element
        psoc.flush()
        
    cv2.imshow("frames", frame)

end_time = time.time()
elapsed_seconds = end_time-start_time
print(str(frameCount/elapsed_seconds)+" fps")
vs.stop()

cv2.destroyAllWindows()
