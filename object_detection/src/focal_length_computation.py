#!/usr/bin/env python
# it uses the lbr_kinect_ros_visual_servoing_focal_length

# import the necessary packages
import rospy
from sensor_msgs.msg import Image
import numpy as np
import imutils
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32MultiArray

class ObjectDetector:
	def __init__(self, name):
		# initialize the needed variables of the class
		
		# the size of the sphere in the scene
		self.object_size = 0.7
		self.object_distance = 3.07049
		self.focal_length = 0
		self.pixel_length = 0

		# define the lower and upper boundaries of the "green" ball in the HSV color space, then initialize the
		# list of tracked points
		self.greenLower = (29, 86, 6)	
		self.greenUpper = (64, 255, 255)

		# initialize the ros node for the class
		rospy.init_node('my_node', anonymous=True)

		# subscribe to "vrep/rgb/image" topic which is published by vrep kinect.
		rospy.Subscriber("/vrep/rgb/image", Image, self.vrep_img_callback)
		rospy.spin()
		
	def vrep_img_callback(self, data):
		# rospy.loginfo("The data height is: ", data.height)
		bridge = CvBridge()
		frame = bridge.imgmsg_to_cv2(data, "rgb8")
		
	
		# resize the frame, blur it, and convert it to the HSV color space
		frame = imutils.resize(frame, width=600)
		# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		# construct a mask for the color "red", then perform a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and initialize the current (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
		cnts_sorted = sorted(cnts, key = cv2.contourArea)
		center = None

		# only proceed if at least one contour was found
		if len(cnts) > 0:

			# find the largest contour in the mask, then use it to compute the minimum enclosing circle and
			# centroid
			x = [None] * len(cnts)
			y = [None] * len(cnts)
			radius = [None] * len(cnts)
			M = [None] * len(cnts)
			center = [None] * len(cnts)
			for i in range(len(cnts)):
				((x[i], y[i]), radius[i]) = cv2.minEnclosingCircle(cnts_sorted[-1-i])
				M[i] = cv2.moments(cnts_sorted[-1-i])
				center[i] = (int(M[i]["m10"] / M[i]["m00"]), int(M[i]["m01"] / M[i]["m00"]))
				cv2.circle(frame, (int(x[i]), int(y[i])), int(radius[i]), (0, 255, 255), 2)
				# here I know that there is just one red circle in the scene
				if i > 0 :
					break
		
		# there is just one object in the scene
		self.pixel_length = self.object_size / (2 * radius[0])
		self.focal_length = (self.object_distance * 2 * radius[0]) / float(self.object_size)
		center_final = center[0]
		print 'focal length', self.focal_length
		print 'diameter', 2 * radius[0]


		if center_final != [None]:
			cv2.circle(frame, center_final, 3, (255, 0, 0), 3)

		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF



# instance of the class
object_detector1 = ObjectDetector("obj_focal_length1")
