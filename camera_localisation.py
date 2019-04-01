# Cameron Brown 2019
# Monash University FYP S1
# This code allows our raspberry pi cameras to detect where they are in relation to
# the dart board plane of motion, so it can send correct target position signals.
# User clicks ends of each axis in video feed

from pivideostream import PiVideoStream
import time
import imutils
import cv2
import numpy as np

clicks = []
mx = 0
my = 0

#this function will be called whenever the mouse is left-clicked
def mouse_callback(event, x, y, flags, params):
    global mx
    global my
    if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 3: # left click
        #store the coordinates of the click event
        clicks.append([x, y])
        print(clicks)
    if event == cv2.EVENT_MOUSEMOVE:
        mx = x
        my = y

vs = PiVideoStream().start()
time.sleep(1) # allow camera to warm up

print("Click both edges of range-of-motion, then click ignore-motion line")
print("On frame press: q = quit, r = reset, s = save")
# loop over the frames of the video
while True:
    # grab the current frame
    frame = vs.read()

    if len(clicks) == 1:
        coords= clicks[0]
        x1 = coords[0]
        y1 = coords[1]
        xavg = int((x1 + mx)/2)
        cv2.line(frame, (xavg, y1), (xavg, my), (0, 0, 255), 1) # vertical line
        cv2.line(frame, (0, my), (640, my), (255, 0, 0), 1) # current horizontal line
        cv2.circle(frame, (x1, y1), 5, (0, 255, 0), -1) # previous click dot
        cv2.line(frame, (0, y1), (640, y1), (0, 255, 0), 1) # previous click line
    elif len(clicks) == 2:
        coords= clicks[0]
        x1 = coords[0]
        y1 = coords[1]
        coords= clicks[1]
        x2 = coords[0]
        y2 = coords[1]
        xavg = int((x1 + x2)/2)
        cv2.line(frame, (xavg, y1), (xavg, y2), (0, 255, 0), 1) # vertical line
        cv2.line(frame, (0, y2), (640, y2), (0, 255, 0), 1) # horizontal line
        cv2.circle(frame, (x1, y1), 5, (0, 255, 0), -1) # click dot
        cv2.circle(frame, (x2, y2), 5, (0, 255, 0), -1) # click dot
        cv2.line(frame, (0, y1), (640, y1), (0, 255, 0), 1) # horizontal line
        cv2.line(frame, (mx, 0), (mx, 480), (0, 0, 255), 1) # vertical line
    elif len(clicks) == 3:
        coords= clicks[0]
        x1 = coords[0]
        y1 = coords[1]
        coords= clicks[1]
        x2 = coords[0]
        y2 = coords[1]
        coords= clicks[2]
        x3 = coords[0]
        xavg = int((x1 + x2)/2)
        cv2.line(frame, (xavg, y1), (xavg, y2), (0, 255, 0), 1) # vertical line
        cv2.line(frame, (0, y2), (640, y2), (0, 255, 0), 1) # horizontal line
        cv2.circle(frame, (x1, y1), 5, (0, 255, 0), -1) # click dot
        cv2.circle(frame, (x2, y2), 5, (0, 255, 0), -1) # click dot
        cv2.line(frame, (0, y1), (640, y1), (0, 255, 0), 1) # horizontal line
        cv2.line(frame, (x3, 0), (x3, 480), (0, 0, 255), 1) # vertical line
        
    cv2.imshow('frame', frame)
    cv2.setMouseCallback('frame', mouse_callback)
    k = cv2.waitKey(5) & 0xFF
    if k == ord("q"):
            break
    if k == ord("s") and len(clicks) == 3:
            coords= clicks[0]
            x1 = coords[0]
            y1 = coords[1]
            coords= clicks[1]
            x2 = coords[0]
            y2 = coords[1]
            coords= clicks[2]
            x3 = coords[0]
            xavg = int((x1 + x2)/2)
            with open('calibration_values.txt', 'w') as calib_file:
                calib_file.write(str(xavg) + "," + str(y1) + "," + str(y2) + "," + str(x3) + '\n')
            print("Saved")
            break
    if k == ord("r"):
        clicks.clear()

vs.stop()
cv2.destroyAllWindows()
