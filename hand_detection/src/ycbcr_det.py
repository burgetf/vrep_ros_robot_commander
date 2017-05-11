#!/usr/bin/env python

# Import needed packages
import rospy
import numpy as np
import cv2

# Capture the video from the cam
cap = cv2.VideoCapture(0)

# Loop over the captured frames to detect and segment the hand
# YCbCr is used as the color space for color_information_based detection
while (True):
	
	# Take a frame from video_capture
	# The captured frame is in BGR color space
	(ret, img) = cap.read()
	
	# Convert the color space from BGR to YCrCb
	img_ycbcr = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)

	# Color channel extractions
	"""
	y_channel = img_ycbcr[:, :, 0]
	cr_channel = img_ycbcr[:, :, 1]
	cb_channel = img_ycbcr[:, :, 2]
	"""
	
	# The range to filter the Cr and Cb channels for skin detection
	# The order is Y, Cr and Cb
	lower_band = np.array([0, 133, 77])
	upper_band = np.array([255, 173, 127])

	# Constructing the binary filter for skin detection
	color_mask = cv2.inRange(img_ycbcr, lower_band, upper_band)
	
	cv2.imshow("Color Mask", color_mask)
	cv2.waitKey(1)
