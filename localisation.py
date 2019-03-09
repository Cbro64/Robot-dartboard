from pivideostream import PiVideoStream
import time
import imutils
import cv2
import numpy as np
 
# if the video argument is None, then we are reading from webcam
vs = PiVideoStream().start()
time.sleep(1)

# loop over the frames of the video
while True:
	# grab the current frame
	frame = vs.read()
	# convert to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

	# define range of red color in HSV
	lower1 = np.array([0,100,10])
	upper1 = np.array([5,255,255])
	lower2 = np.array([170,100,10])
	upper2 = np.array([180,255,255])

	# make threshold mask
	mask = cv2.inRange(hsv, lower1, upper1) + cv2.inRange(hsv, lower2, upper2)
	mask = cv2.dilate(mask, None, iterations=2)

	#M = cv2.moments(thresh)
	#if M['m00']:
	#	cx = int(M['m10']/M['m00'])
	#	cy = int(M['m01']/M['m00'])
	#	#cv2.circle(totalDiff, (cx, cy), 4, (0,0,255))

	#find contours
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
 
	#loop over the contours and find 2 largest
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
			
	if marker1size > 0 and marker2size > 0:
		M = cv2.moments(marker1)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
		cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)
		cv2.drawContours(frame, [marker1, marker2], -1, (0, 0, 255), 2)
 
	cv2.imshow('frame',frame)
	cv2.imshow('mask',mask)
	#cv2.imshow('res',res)
	k = cv2.waitKey(5) & 0xFF
	if k == ord("q"):
        	break


vs.stop()
cv2.destroyAllWindows()