#!/usr/bin/env python

# import the necessary packages
import rospy
import numpy as np
import cv2

# video capture
cap = cv2.VideoCapture(0)

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
fgbg = cv2.createBackgroundSubtractorGMG()

while(True):
	ret, frame = cap.read()
	# frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	fgmask = fgbg.apply(frame, learningRate=0.001)
	fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)

	cv2.imshow('frame',fgmask)
	k = cv2.waitKey(30) & 0xff
	if k == 27:
		break
 
cap.release()
cv2.destroyAllWindows()


