#!/usr/bin/env python

# import the necessary packages
import rospy
import numpy as np
import cv2
import math
import imutils
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vrep_common.msg import DetectedObjInf

class HandDetector:
	def __init__(self, name):
		# some needed variables
		self.img_height = 480
		self.img_width = 640
		self.img_ycrcb = np.empty((self.img_height, self.img_width))
		self.color_mask = np.empty((self.img_height, self.img_width))
		self.depth_mask = np.empty((self.img_height, self.img_width))
		self.final_mask = np.empty((self.img_height, self.img_width))
		self.lower_band = np.array([0, 133, 77])
		self.upper_band = np.array([255, 173, 127])
		self.kernel = np.ones((3,3), dtype=np.uint8)
		self.depth_thresh = 0.1
		self.palm_coordinate = None

		# the estimated focal_length(pixels) of the kinect sensor and the sphere diameter(m) in vrep
		self.focal_length = 573.5
		# self.obj_size = 0.1

		# background subtractor definition
		# self.back_sub = cv2.BackgroundSubtractorMOG2(history = 1, varThreshold=16, bShadowDetection=False)
		
		
		# initialize the ros node
		rospy.init_node(name, anonymous = False)

		# Initialize the publisher for detected hand information
		self.obj_inf_pub = rospy.Publisher("object_detection/obj_inf", DetectedObjInf, queue_size=1)

		# Initialize the subscriber to rgb image topic
		rospy.Subscriber("/camera/rgb/image_raw", Image, self.color_cb)
				
		# Initialize the subscriber to depth image topic
		rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_cb)

		rospy.spin()

	def color_cb(self, data):	
		try:
			# Get the image in cv2 format
			bridge = CvBridge()
			img = bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
	
		# Convert the color space of image from BGR to YCrCb
		self.img_ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
	 
		# Get the color_mask based on lower and upper band for skin detection
		self.color_mask = cv2.inRange(self.img_ycrcb, self.lower_band, self.upper_band)

		# apply medianblurring over the mask
		self.color_mask = cv2.medianBlur(self.color_mask, 5)

		# extract binary mask based on the threshold
		ret, self.color_mask = cv2.threshold(self.color_mask,127,255,0)

		# cv2.imshow("color_frame", self.color_mask)
		# cv2.waitKey(1)

	def depth_cb(self, data):
		try:
			# Get the image in cv2 format
			bridge = CvBridge()
			img = bridge.imgmsg_to_cv2(data, "passthrough")
	
		except CvBridgeError as e:
			print(e)

		input_depth_img = np.array(img, dtype=np.float32)
		input_depth_img[np.isnan(input_depth_img)] = 3.0
		depth_img = input_depth_img.copy()
		cv2.normalize(depth_img, depth_img, 0, 1, cv2.NORM_MINMAX)

		# Application of median filtering to remove possible noises in depth values
		img_blurred = cv2.medianBlur(depth_img, 5)
		
		# Application of close operation (dilation + erosion)
		img_dilated = cv2.dilate(img_blurred, self.kernel, iterations=1)
		img_eroded = cv2.erode(img_dilated, self.kernel, iterations=1)
	
		# Filter the depth image by depth threshold
		img_filtered = cv2.inRange(img_eroded, 0, self.depth_thresh)
		# cv2.normalize(img_filtered, img_filtered, 0, 1, cv2.NORM_MINMAX)

		# Apply final dilation
		self.depth_mask = cv2.dilate(img_filtered, self.kernel, iterations=1)

		# Construct the final_mask using depth_mask and color_mask
		self.final_mask = cv2.bitwise_and(self.depth_mask, self.color_mask)

		# cv2.imshow("final_frame", self.final_mask)
		# cv2.waitKey(1)


		### applying the morphological transformations to get a smoother mask and get rid of background noise
		square_kernel = np.ones((7, 7), np.uint8)

		# structuring element for morphological transformations
		ellipse_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
	
		# erosion discards the extra boundary pixels
		# dilation extends the boundary
		# median_blurring is effective in salt-and-pepper noise
		# squart_kernel is used for erosion and ellipse_kernel is used for dilation
		mask_transformed = cv2.dilate(self.final_mask, ellipse_kernel, iterations=1)
		# cv2.imshow("transform1", mask_transformed)
		mask_transformed = cv2.erode(mask_transformed, square_kernel, iterations=1)
		# cv2.imshow("transform2", mask_transformed)
		mask_transformed = cv2.dilate(mask_transformed, ellipse_kernel, iterations=1)
		# cv2.imshow("transform3", mask_transformed)
		mask_blurred = cv2.medianBlur(mask_transformed, 5)
		# cv2.imshow("blur1", mask_blurred)
 
		# changing the size of kernel to get a better result
		# ellipse_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
		# mask_transformed = cv2.dilate(mask_blurred, ellipse_kernel, iterations=1)
		# mask_blurred = cv2.medianBlur(mask_transformed, 3)

		(retval, mask_thresh) = cv2.threshold(mask_blurred, 127, 255, 0)

		# contour detection over the mask
		(img_contours, hierarchy) = cv2.findContours(mask_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		img_contours_sorted = sorted(img_contours, key = cv2.contourArea)
		if len(img_contours_sorted) > 0:
			img_largest_contour = img_contours_sorted[-1]
		else:
			img_largest_contour = None
			return

		# find the convex hull for the largest contour
		# conxexHull() takes points
		convex_hull = cv2.convexHull(img_largest_contour)

		# drawContours() takes image, contours, contourIdx: identify which contours to draw (negative -> all contours), color and thickness
		# green contour, blue convex_hull
		drawing = np.zeros(img.shape, np.uint8)
		cv2.drawContours(self.img_ycrcb, [img_largest_contour], 0, (255, 0, 0), 2)
		cv2.drawContours(self.img_ycrcb, [convex_hull], 0, (0, 0, 255), 2)

		
		# finding the defect points in convex hull
		# approxPolyDP() approximates a polygonal curve with the specified percision
		# approxPolyDP() take curve, epsilon: accuracy and closed: closed curve? (True/False)
		approx_contour = cv2.approxPolyDP(img_largest_contour, 0.01*cv2.arcLength(img_largest_contour,True), True)

		# conxexHull() takes points and returnPoints: if True it returns the convex_hull_points, otherwise their indices in contour array
		convex_hull_2 = cv2.convexHull(approx_contour, returnPoints = False)

		# convexityDefects's output is a collection of elements. Each element has 4 values: start_index, end_index, farthest_point_index and fixpoint_depth
		defect_points = cv2.convexityDefects(approx_contour, convex_hull_2)
		if (defect_points == None):
			defect_points = np.array([])

		if (len(defect_points) > 0):
			palm_elements = np.zeros((len(defect_points), 2))
			# finger_lengths = np.zeros(len(defect_points))

		num_finger_defects = 0

		for i in range(len(defect_points)):
			si, ei, fpi, fd = defect_points[i,0]
			start = tuple(approx_contour[si][0])
			end = tuple(approx_contour[ei][0])
			# mid = tuple([sum(x)/2 for x in zip(start, end)])
			farthest_point = tuple(approx_contour[fpi][0])

			# blue finger tips
			cv2.circle(self.img_ycrcb, start, 5, (255, 0, 0), thickness = 3)
		

			# pointPolygonTest() takes contour, point and measureDist: if true the function measures the distance between the point and the nearest contour otherwise just checks if the point is inside the contour or not.
			# dist = cv2.pointPolygonTest()

			# checking the angle between the fingers
			if self.angle_cal(start, end, farthest_point) < np.pi * 90/180:
				num_finger_defects += 1
				# yellow finger bases
				cv2.circle(self.img_ycrcb, farthest_point, 5, (0, 255, 255), thickness = 3)
				palm_elements[i] = np.array(farthest_point)
				# finger_lengths[i] = np.sqrt(sum((np.array(start) - np.array(farthest_point)) ** 2))

		if len(defect_points > 4):
			publish_available = True
		else:
			publish_available = False

		# needed data to publish
		if publish_available:
			palm_points = palm_elements[:num_finger_defects]	
			# longest_finger = np.max(finger_lengths)
			palm_center = np.mean(palm_points, axis=0)
			palm_marker = tuple([int(palm_center[0]), int(palm_center[1])])
			Z = input_depth_img[palm_marker[1], palm_marker[0]]
			print "hand distance to cam", Z
			cv2.circle(self.img_ycrcb, tuple(palm_marker), 3, (255, 0, 0), thickness=3)
			# hand_dist = (self.obj_size * self.focal_length) / float(longest_finger)
			# pixel_size = self.obj_size / float(longest_finger)

		# calculated number of fingers
		# print num_finger_defects

		cv2.imshow("drawing:", self.img_ycrcb)
		k = cv2.waitKey(1)
		
		# publish the pose of detected hand
		if publish_available:
			msg_to_publish = DetectedObjInf()
			msg_to_publish.isValid.data = 1
			msg_to_publish.u.data = int(palm_elements[0, 0])
			msg_to_publish.v.data = int(palm_elements[0, 1])
			msg_to_publish.dist.data = Z
			msg_to_publish.pixelSize.data = 0
			self.obj_inf_pub.publish(msg_to_publish)
		else:
			msg_to_publish = DetectedObjInf()
			msg_to_publish.isValid.data = 0
			msg_to_publish.u.data = 0
			msg_to_publish.v.data = 0
			msg_to_publish.dist.data = 0
			msg_to_publish.pixelSize.data = 0
			self.obj_inf_pub.publish(msg_to_publish)

	def angle_cal(self, end1, end2, base):
		delta1 = [a-b for a, b in zip(end1, base)]
		delta2 = [a-b for a, b in zip(end2, base)]
		angle1 = math.atan2(delta2[1], delta2[0]) 
		angle2 = math.atan2(delta1[1], delta1[0])  
		final_angle = angle2 - angle1
		return final_angle

# instance of the class
hand_detector1 = HandDetector("hand_det1")

