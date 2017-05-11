#!/usr/bin/env python

# import the necessary packages
import rospy
import numpy as np
import cv2

# video capture
cap = cv2.VideoCapture(0)

# default value of parameters are: 
# history=200, nmixtures=5, backgroundRatio=0.7
fgbg = cv2.BackgroundSubtractorMOG(history = 5, nmixtures=5, backgroundRatio=0.2)

while(True):
	ret, frame = cap.read()
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	fgmask = fgbg.apply(frame, learningRate=0.001)
	
	# invert the mask. foreground will be 255 and background 0
	fgmask = 255 - fgmask

	cv2.imshow('frame',fgmask)
	k = cv2.waitKey(30) & 0xff
 
cap.release()
cv2.destroyAllWindows()
