# Robotic dartboard raspberry pi code for final year project
# Monash University
# Cameron Brown 2018
# 
# Frame capture method adapted from 
# https://gist.github.com/CarlosGS/b8462a8a1cb69f55d8356cbb0f3a4d63
# @CarlosGS May 2017

import cv2
import numpy as np
import subprocess as sp
import time
import atexit
import serial
import os

maxFrames = 25

sendRangeMax = 255;
motionThreshold = 0.15 # percentage of the frame
ignore = 10 # ignore first number of frames while camera warms up

# open serial port
ser = serial.Serial('/dev/ttyACM0',9600)



# Video capture parameters
(w,h) = (640,480)
bytesPerFrame = w * h
fps = 35 # setting to 250 will request the maximum framerate possible
# image processing will slow down the pipeline, so the requested FPS should be set just below the pipeline speed
motionThreshold = motionThreshold * w * h * 255 / 100 # convert from percent to pixel value sum

# read calibration values from text file or use default
xPlane = 0
y1 = 0
y2 = h
with open('calibration_values.txt', 'r') as calib_file:
    params = calib_file.read().split(',')
    xPlane = int(params[0])
    y1 = int(params[1])
    y2 = int(params[2])

if(y1 > y2):
    temp = y2
    y2 = y1
    y1 = temp
range2 = y2-y1
y1 += range2*0.1
y2 -= range2*0.1

print(y1)
print(y2)
print(xPlane)
# "raspividyuv" is the command that provides camera frames in YUV format
#  "--output -" specifies stdout as the output
#  "--timeout 0" specifies continuous video
#  "--luma" discards chroma channels, only luminance is sent through the pipeline
# see "raspividyuv --help" for more information on the parameters
videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --luma --nopreview -md 4" #-md 4 for full fov or -md 7 for more fps
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string
cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE, bufsize=0) # start the camera
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly
cv2.waitKey(6000) # wait for camera to warm up

throwNumber = 0
#start_time = time.time() # timing is temporarily removed
while True:
    throwNumber+=1
    cv2.waitKey(3000)
    print("Ready to throw!")
    frames = [] # stores the frames for later review
    threshFrames = []
    frameCount = 0
    lastFrame = None
    dartThrown = False
    pathx = []
    pathy = []

    # Capture loop
    while True:
        cameraProcess.stdout.flush() # discard any frames that we were not able to process in time
        # Parse the raw stream into a numpy array
        frame = np.fromfile(cameraProcess.stdout, count=bytesPerFrame, dtype=np.uint8)
        if frame.size != bytesPerFrame:
            print("Error: Camera stream closed unexpectedly")
            break
        frame.shape = (h,w) # set the correct dimensions for the numpy array
        if ignore > 0:
            ignore -= 1
            continue
        #frame = cv2.GaussianBlur(frame, (5, 5), 0)

        if lastFrame is None:
            lastFrame = frame

        frameDiff = cv2.absdiff(lastFrame, frame)
        thresh = cv2.threshold(frameDiff, 10, 255, cv2.THRESH_BINARY)[1]

        if not dartThrown:
            motion = np.sum(thresh)
            #print(motion/255/(w*h)*100)
            if motion > motionThreshold:
                dartThrown = True
            else:
                cv2.imshow("frames",frame)
                # exit if 'q' key is pressed
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

        # dart throw started
        if dartThrown:
            frameCount += 1

            # Calculate center of motion 
            M = cv2.moments(thresh)
            if M['m00']:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                pathx.append(cx) # save tracking coords
                pathy.append(cy)
                #cv2.circle(thresh, (cx, cy), 4, (0,0,255))


            path = np.polyfit(pathx, pathy, 1) # fit line to coords
            p = np.poly1d(path)
            targetY = int(p(xPlane)); # get y-value prediction at x plane
            cv2.circle(frame, (xPlane, targetY), 4, (255,255,255))

            # send prediction to arduino
            # remember to end write messages with \r\n
            #print(targetY)
            if frameCount >= 3:
                cv2.circle(frame, (xPlane, targetY), 4, (255,255,255))
                if targetY < y1:
                    sendY = 0
                elif targetY > y2:
                    sendY = sendRangeMax
                else:
                    sendY = int(float((targetY - y1)) / float((y2 - y1)) * sendRangeMax)
                #print("Sending: " + str(sendY));
                #print("Prediction sent on frame: " +str(frameCount))
                ser.write(str.encode(str(sendY)) + b'\r\n')


            frames.append(frame) # save the frame
            threshFrames.append(thresh)

            if frameCount > maxFrames:
                break

        lastFrame = frame



    #end_time = time.time()
    #elapsed_seconds = end_time-start_time

    #print("Done! Result: "+str(frameCount/elapsed_seconds)+" fps")
    print("Throw captured")

    save = 1
    if save:
        print("Writing frames to disk...")
    #    out = cv2.VideoWriter("slow_motion.avi", cv2.VideoWriter_fourcc(*"MJPG"), 30, (w,h))
        os.mkdir(str(throwNumber))
        for n in range(N_frames):
            cv2.imwrite(str(throwNumber) + "/frame"+str(n)+".png", frames[n]) # save frame as a PNG image
     #       frame_rgb = cv2.cvtColor(frames[n],cv2.COLOR_GRAY2RGB) # video codec requires RGB image
     #       out.write(frame_rgb)
     #   out.release()

    display = 1
    if display & (len(frames) > 0):
        key = "f"
        print("Displaying frames")
        cv2.waitKey(2000)
        while(key != ord("q")):
            for i in range(len(frames)):
                print("Showing frame number: " + str(i+1))
                cv2.imshow("frames", frames[i])
                cv2.imshow("thresh", threshFrames[i])
                key = cv2.waitKey(0) & 0xFF # time between frames in ms (0 = keypress only)
                if key == ord("q"):
                    break
                
cameraProcess.terminate() # stop the camera
cv2.destroyAllWindows()
