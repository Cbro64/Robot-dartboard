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
import struct


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
t = [1.175, 0.045, 1.243]  # translation in new frame orientation
T = np.array([[0, 0, -1, t[0]], [0, -1, 0, t[1]], [-1, 0, 0, t[2]]])
T = T.T # transpose, used in frame loop

c1 = np.array([1.175,0.045,1.243])
c2 = np.array([0,-1.575,1.2])
deltaC = np.subtract(c2, c1)
#print(deltaC)

#  Start camera process
# "raspividyuv" is the command that provides camera frames in YUV format
#  "--output -" specifies stdout as the output
#  "--timeout 0" specifies continuous video
#  "--luma" discards chroma channels, only luminance is sent through the pipeline
videoCmd = "raspividyuv -w " + str(w) + " -h " + str(h) + " --output - --timeout 0 --framerate " + str(
    fps) + " --luma --nopreview -md 4"  # -md 4 for full fov or -md 7 for more fps
videoCmd = videoCmd.split()  # Popen requires that each parameter is a separate string
cameraProcess = sp.Popen(videoCmd, stdout=sp.PIPE, bufsize=0)  # start the camera
atexit.register(cameraProcess.terminate)  # this closes the camera process in case the python scripts exits unexpectedly
cv2.waitKey(1000)  # wait for camera to warm up

lastFrame = None
capture = False

xCutoff = 630  # ignore pixels past this value in horizontal direction
motionThreshold = 0.05 #% percentage of frame required for motion detection
motionThreshold = motionThreshold * w * h / 100 # convert to number of pixels
frameCount = 0
start_time = 0
pointCounter = 0
pointsX = []
pointsY = []
pointsZ = []
# Capture loop
while True:
    cameraProcess.stdout.flush()  # discard any frames that we were not able to process in time
    # Parse the raw stream into a numpy array
    frame = np.fromfile(cameraProcess.stdout, count=bytesPerFrame, dtype=np.uint8)
    if frame.size != bytesPerFrame:
        print("Error: Camera stream closed unexpectedly")
        break
    frame.shape = (h, w)  # set the correct dimensions for the numpy array

    if lastFrame is None:
        lastFrame = frame
        
        continue
    while psoc.in_waiting:
            print('skipping')
            x = psoc.read(12)
    frameCount += 1

    frameDiff = cv2.absdiff(lastFrame, frame)
    thresh = cv2.threshold(frameDiff, 10, 255, cv2.THRESH_BINARY)[1]
    thresh[:, xCutoff:w] = 0  # assumes target is on right hand side of frame
    lastFrame = frame.copy()

    # Exit if 'q' key is pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

    if key == ord("s"):
        start_time = time.time()
        capture = True
    
    
    if capture != True:
        cv2.imshow("frames", frame)
        continue
    
    
    frameCount += 1
    
    if np.sum(thresh) < motionThreshold:
        continue
    
    # Calculate center of motion
    M = cv2.moments(thresh)
    if M['m00']:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.circle(frame, (cx, cy), 4, (0,0,255))

    # Convert from pixel coords to frame coords
        pix = np.array([cx, cy, 1])
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
        #print(list(V.astype('f').tostring()))
        #psoc.write(V.astype('f').tostring()) # 12 bytes, 4 per element
        '''
        float f;
        uchar b[] = {b0, b1, b2, b3};
        memcpy( & f, & b, sizeof(f));
        '''
        psoc.flush()
        print('waiting')
        V2 = []
        while psoc.in_waiting:
            print('reading')
            x = psoc.read(12)
            print(list(x))
            V2 = np.frombuffer(x,dtype=np.float32)
        if len(V2) == 0:
            continue
        print(V1)
        print(V2)
        V = np.stack((V1, -V2))
        s = np.dot(np.dot(np.linalg.inv(np.dot(V, V.T)),V),deltaC)
        p1 = c1 + s[0]*V1
        p2 = c2 + s[1]*V2
        p = (p1+p2)/2
        pointCounter += 1
        print(pointCounter)
        pointsX.append(p[0])
        pointsY.append(p[1])
        pointsZ.append(p[2])
        print(p)
        #if(pointCounter == 4):
        if pointCounter == 4:
            pathVert = np.polyfit(pointsY, pointsZ, 2) # fit line to coords
            pathHori = np.polyfit(pointsX, pointsZ, 1)
            pV = np.poly1d(pathVert)
            pH = np.poly1d(pathHori)
            targetY = pV(0) # get y-value prediction
            targetX = pH(0) # get x-value prediction
            print("X prediction: "+str(targetX))
            print("Y prediction: "+str(targetY))
            #cv2.imshow("frames", frame)
            #cv2.imshow("threshold", thresh)
            break
        #capture = False
        #cv2.imshow("threshold", thresh)
        #cv2.imshow("frames", frame)
        #while True:
        #    key = cv2.waitKey(1) & 0xFF
        #    cv2.imshow("frames", frame)
        
       
    #cv2.imshow("frames", frame)
    #cv2.imshow("threshold", thresh)

print(pointCounter)
end_time = time.time()
elapsed_seconds = end_time-start_time
print(str(frameCount/elapsed_seconds)+" fps")
cv2.imshow("frames", frame)
cv2.waitKey(100000)

cameraProcess.terminate()  # stop the camera


cv2.destroyAllWindows()