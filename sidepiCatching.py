# Robotic dartboard raspberry pi code for final year project
# Monash University
# Cameron Brown 2019
# 
# Fast frame grabbing method adapted from
# https://gist.github.com/CarlosGS/b8462a8a1cb69f55d8356cbb0f3a4d63
# @CarlosGS May 2017

import cv2
import numpy as np
import subprocess as sp
import time
import atexit
import serial
import os

maxFrames = 64 # number of frames to capture per throw

sendRangeMax = 250 # range to use for predictions (0-250)
motionThreshold = 0.03 # percentage of the frame that must contain motion to detect start of throw

# open serial port
ser = serial.Serial('/dev/ttyS0',57600)

# Video capture parameters
(w,h) = (320,192)
bytesPerFrame = w * h
fps = 250 # setting to 250 will request the maximum framerate possible
motionThreshold = motionThreshold * w * h * 255 / 100 # convert from percent to pixel value sum

# read corner points of catch zone from text file
bounds=[]
with open('calibration_values.txt', 'r') as calib_file:
    params = calib_file.read().split(',')
    print(len(params))
    for point in range(4):
        bounds.append([int(params[2*point]), int(params[2*point+1])])
    xCutoff = int(params[8])
xPlane = int(((int(bounds[0][0]) + int(bounds[2][0])) / 2)) # set x prediction point to center of plane to start with
xPlane2 = xPlane
y1init = int(((int(bounds[0][1]) + int(bounds[2][1])) / 2)) # y bounds of catch zone - upper
y2init = int(((int(bounds[1][1]) + int(bounds[3][1])) / 2)) # lower

y1 = y1init
y2 = y2init

topHorGap = (int(bounds[2][0]) - int(bounds[0][0])) # horizontal gap between top corners of catch plane
botHorGap = (int(bounds[3][0]) - int(bounds[1][0])) # horizontal gap between bottom corners of catch plane
topVerGap = (int(bounds[0][1]) - int(bounds[2][1])) # vertical gap between top corners of catch plane
botVerGap = (int(bounds[3][1]) - int(bounds[1][1])) # vertical gap between bottom corners of catch plane

start_time = 0
elapsed_seconds = 0
throwNumber = 0
stop = False

# "raspividyuv" is the command that provides camera frames in YUV format
#  "--output -" specifies stdout as the output
#  "--timeout 0" specifies continuous video
#  "--luma" discards chroma channels, only luminance is sent through the pipeline
# see "raspividyuv --help" for more information on the parameters
videoCmd = "raspividyuv -w "+str(w)+" -h "+str(h)+" --output - --timeout 0 --framerate "+str(fps)+" --luma --nopreview -md 6" #-md 4 for full fov or -md 6 for more fps
videoCmd = videoCmd.split() # Popen requires that each parameter is a separate string
cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE, bufsize=0) # start the camera
atexit.register(cameraProcess.terminate) # this closes the camera process in case the python scripts exits unexpectedly
cv2.waitKey(2000) # wait for camera to warm up

# wait for 'homing finished' message from motion system
while (ser.read(1)[0] != 251):
    nothing =1
print('Homing finished')

# loop catching procedure until program is exited
while True:
    throwNumber+=1
    cameraProcess.stdout.flush()
    
    frames = [] # stores the frames for later review
    threshFrames = [] # stores the threshold frames for later review
    frameCount = -1 # start at -1 because two frames are needed for first frame diff
    lastFrame = None
    dartThrown = False
    pathx = []  # stores x pixel coords of detected projectile
    pathy = []  # stores y pixel coords of detected projectile
    ignore = 10  # ignore first 10 frames to allow camera settings to stabilise (brightness etc)

    # Tell floor pi to get ready to start throw
    print("Ready to throw!")
    startByte = 252
    ser.write(startByte.to_bytes(1, 'little'))
    ser.flush()
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
            lastFrame = frame.copy()
            continue

        frameDiff = cv2.absdiff(lastFrame, frame)
        #frameDiff = cv2.subtract(frame, lastFrame)
        thresh = cv2.threshold(frameDiff, 10, 255, cv2.THRESH_BINARY)[1]
        thresh[:,xCutoff:w] = 0 # assumes target is on right hand side of frame
        
        lastFrame = frame.copy()

        #unfinished windowing code that follows projectile
        #if frameCount >= 3:
        #    ywindow = max(abs(pathy[-1] - pathy[-2])*3,30)
        #   xwindow = max(abs(pathx[-1] - pathx[-2])*4,50)
        #    thresh[:,0:pathx[-1]-20] = 0
        #   thresh[:,pathx[-1]+xwindow:w] = 0
        #   thresh[0:pathy[-1]-ywindow,:] = 0
        #   thresh[pathy[-1]+ywindow:h,:] = 0
        #   cv2.line(frame, (pathx[-1], 0), (pathx[-1], h), (255, 255, 255), 1)
        #   cv2.line(frame, (pathx[-1]+xwindow, 0), (pathx[-1]+xwindow, h), (255, 255, 255), 1)
        #   cv2.line(frame, (0, pathy[-1]-ywindow), (w, pathy[-1]-ywindow), (255, 255, 255), 1)
        #   cv2.line(frame, (0, pathy[-1]+ywindow), (w, pathy[-1]+ywindow), (255, 255, 255), 1)
        #   cv2.circle(frame, (pathx[-1], pathy[-1]), 4, (255,255,255))

        if not dartThrown:
            #print(motion/255/(w*h)*100)
            motion = np.sum(thresh)
            if motion > motionThreshold:
                dartThrown = True
                start_time = time.time()

            else:
                cv2.imshow("frames",frame)
                # exit if 'q' key is pressed
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    stop = True
                    break
        if dartThrown:
            frames.append(frame) # save the frame
            # draw catch zone on frame
            cv2.line(frame, (bounds[0][0], bounds[0][1]), (bounds[1][0], bounds[1][1]), (255, 255, 255), 1)
            cv2.line(frame, (bounds[0][0], bounds[0][1]), (bounds[2][0], bounds[2][1]), (255, 255, 255), 1)
            cv2.line(frame, (bounds[1][0], bounds[1][1]), (bounds[3][0], bounds[3][1]), (255, 255, 255), 1)
            cv2.line(frame, (bounds[2][0], bounds[2][1]), (bounds[3][0], bounds[3][1]), (255, 255, 255), 1)
            threshFrames.append(thresh)
            frameCount += 1
        
        # dart throw started
        if dartThrown:# and motion>motionThreshold:
            
            # Calculate center of motion 
            M = cv2.moments(thresh)
            if M['m00']:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                pathx.append(cx) # save tracking coords
                pathy.append(cy)
                #cv2.circle(thresh, (cx, cy), 4, (0,0,255))

                
            # send prediction to psoc
            #print(targetY)
            if frameCount >= 10: # wait for 10 points before sending first prediction
                path = np.polyfit(pathx, pathy, 2) # fit quad to coords
                p = np.poly1d(path)
                for i in range(3):
                    targetY = int(p(xPlane2)); # get y-value prediction at x plane
                    cv2.circle(frame, (xPlane2, targetY), 3, (255,255,255))
                    #cv2.circle(frame, (xPlane2, y1), 3, (255,255,255))
                    #cv2.circle(frame, (xPlane2, y2), 3, (255,255,255))
                    cv2.line(frame, (xPlane2, 0), (xPlane2, h), (255, 255, 255), 1)

                    # limit prediction to inside catch zone
                    if targetY < y1:
                        sendY = sendRangeMax
                    elif targetY > y2:
                        sendY = 0
                    else:
                        # convert pixel coordinate to 0-250
                        sendY = sendRangeMax - int(float((targetY - y1)) / float((y2 - y1)) * sendRangeMax)
                    print("Sending: " + str(sendY));
                    #print("Prediction sent on frame: " +str(frameCount))
                    ser.write(sendY.to_bytes(1, 'little'))
                    #ser.flush()
                    #wait for prediction from other pi
                    print("waiting for other pi")
                    other = ser.read(1)[0]
                    #print(other)
                    #other = 0

                    # move x coord prediction line based on info from other pi
                    offset = int(-topHorGap/2 + other/sendRangeMax*topHorGap)
                    xPlane2 = xPlane + offset

                    # fix upper limits of catch zone based on info from other pi
                    y1 = y1init + int(-topVerGap/2 + (sendRangeMax-other)/sendRangeMax*topVerGap)
                    y2 = y2init - int(-botVerGap/2 + (sendRangeMax-other)/sendRangeMax*botVerGap)

            #frames.append(frame) # save the frame
            #threshFrames.append(thresh)

        if frameCount >= maxFrames: #or (dartThrown and motion < motionThreshold):
            end_time = time.time()
            elapsed_seconds = end_time-start_time
            print("FPS: "+str(frameCount/elapsed_seconds)+" fps")
            # Tell floor pi and psoc that throw has finished
            stopByte = 253
            ser.write(stopByte.to_bytes(1, 'little'))
            ser.flush()
            break
    
    if stop:
        break
    print("Throw captured")

    save = 1 # toggle to save frames to disk
    if save:
        print("Writing frames to disk...")
    #    out = cv2.VideoWriter("slow_motion.avi", cv2.VideoWriter_fourcc(*"MJPG"), 30, (w,h))
        os.mkdir(str(throwNumber))
        for n in range(maxFrames):
            cv2.imwrite(str(throwNumber) + "/frame"+str(n)+".png", frames[n]) # save frame as a PNG image
            cv2.imwrite(str(throwNumber) + "/thresh"+str(n)+".png", threshFrames[n])
    #       frame_rgb = cv2.cvtColor(frames[n],cv2.COLOR_GRAY2RGB) # video codec requires RGB image
    #       out.write(frame_rgb)
    #   out.release()

    
    display = 1 # toggle to display frames after throw
    if display & (len(frames) > 0):
        key = "f"
        print("Displaying frames")
        cv2.waitKey(2000)
        while(key != ord("q") and key != ord("n")):
            for i in range(len(frames)):
                print("Showing frame number: " + str(i+1))
                cv2.imshow("frames", frames[i])
                cv2.imshow("thresh", threshFrames[i])
                key = cv2.waitKey(0) & 0xFF # time between frames in ms (0 = keypress only)
                if key == ord("n"):
                    break
                if key == ord("q"):
                    stop = True
                    break
    if stop:
        break
    print("Next throw")

cameraProcess.terminate() # stop the camera
cv2.destroyAllWindows()
