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

# This function will be called whenever the mouse is left-clicked
def mouse_callback(event, x, y, flags, params):
    global mx
    global my
    if event == cv2.EVENT_LBUTTONDOWN and len(clicks) < 5: # left click
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
    for point in clicks:
        cv2.circle(frame, (point[0], point[1]), 3, (255, 0, 0), -1)
    
    if len(clicks) > 3:
        # draw green lines outline catch zone
        cv2.line(frame, (clicks[0][0], clicks[0][1]), (clicks[1][0], clicks[1][1]), (0, 255, 0), 1)
        cv2.line(frame, (clicks[0][0], clicks[0][1]), (clicks[2][0], clicks[2][1]), (0, 255, 0), 1)
        cv2.line(frame, (clicks[2][0], clicks[2][1]), (clicks[3][0], clicks[3][1]), (0, 255, 0), 1)
        cv2.line(frame, (clicks[1][0], clicks[1][1]), (clicks[3][0], clicks[3][1]), (0, 255, 0), 1)
        # blue lines indicating centre
        cv2.line(frame, (int((clicks[0][0]+clicks[2][0])/2), 0), (int((clicks[0][0]+clicks[2][0])/2), 480), (255, 0, 0), 1)
        cv2.line(frame, (0, int((clicks[0][1]+clicks[1][1])/2)), (640, int((clicks[0][1]+clicks[1][1])/2)), (255, 0, 0), 1)
    
    if len(clicks) == 4:
        cv2.line(frame, (mx, 0), (mx, 480), (0, 0, 255), 1) # vertical line for motion ignore zone
    if len(clicks) >= 5:
        cv2.line(frame, (clicks[4][0], 0), (clicks[4][0], 480), (0, 255, 0), 1)   
        
    cv2.imshow('frame', frame)
    cv2.setMouseCallback('frame', mouse_callback)
    k = cv2.waitKey(5) & 0xFF
    if k == ord("q"):
            break
    if k == ord("s") and len(clicks) == 5:
            # save points to text file
            with open('calibration_values.txt', 'w') as calib_file:
                for point in clicks:
                    calib_file.write(str(point[0]) + "," + str(point[1]) + ",")
            print("Saved")
            break
    if k == ord("r"):
        clicks.clear()

vs.stop()
cv2.destroyAllWindows()
