#!/usr/bin/env python

# import the necessary packages
import rospy
from sensor_msgs.msg import Image
from vrep_common.msg import DetectedObjInf
import numpy as np
import imutils
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32MultiArray, Float64

class ObjectDetector:
	def __init__(self, name):
		# initialize the needed variables of the class

		# define the lower and upper boundaries of the "green" ball in the HSV color space

		self.greenLower = (29, 86, 6)
		self.greenUpper = (64, 255, 255)

		# the estimated focal_length(pixels) of the kinect sensor and the sphere diameter(m)
		self.focal_length = 573.5
		self.obj_size = 0.055

		# initialize the ros node for the class
		rospy.init_node(name)

		# initialize the publisher for the object center and distance
		self.obj_inf_pub = rospy.Publisher("object_detection/obj_inf", DetectedObjInf, queue_size=1)

		# subscribe to image topic which is published
		# rospy.Subscriber("/vrep/rgb/image", Image, self.img_callback)
		rospy.Subscriber("/camera/rgb/image_raw", Image, self.img_callback)

		rospy.spin()
		
	def img_callback(self, data):

		# rospy.loginfo("The data height is: ", data.height)
		bridge = CvBridge()
		frame = bridge.imgmsg_to_cv2(data, "rgb8")
	
		# resize the frame, blur it, and convert it to the HSV color space
		frame = imutils.resize(frame, width=600)
		# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		# construct a mask for the color "green", then perform a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and initialize the current (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

		# only proceed if at least one contour was found
		if len(cnts) > 0:
			cnts_sorted = sorted(cnts, key = cv2.contourArea)
			largest_contour = cnts_sorted[-1]

			# find the largest contour in the mask, then use it to compute the minimum enclosing circle and
			# centroid
			((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
			M = cv2.moments(largest_contour)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			if (radius>10):
				cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
				publish_available = True
			else: 
				publish_available = False
		else:
			publish_available = False

		cv2.imshow("Ball detection frame", frame)
		key = cv2.waitKey(1) & 0xFF

		# publish the center and orientation of detected object
		if publish_available:
			obj_dist = (self.obj_size * self.focal_length) / float(2 * radius)
			pixel_size = self.obj_size / float(2 * radius)
			msg_to_publish = DetectedObjInf()
			msg_to_publish.isValid.data = 1
			msg_to_publish.u.data = center[0]
			msg_to_publish.v.data = center[1]
			msg_to_publish.dist.data = obj_dist
			msg_to_publish.pixelSize.data = pixel_size
			self.obj_inf_pub.publish(msg_to_publish)
		else:
			msg_to_publish = DetectedObjInf()
			msg_to_publish.isValid.data = 0
			msg_to_publish.u.data = 0
			msg_to_publish.v.data = 0
			msg_to_publish.dist.data = 0
			msg_to_publish.pixelSize.data = 0
			self.obj_inf_pub.publish(msg_to_publish)
			

# instance of the class
object_detector1 = ObjectDetector("obj_det1")







