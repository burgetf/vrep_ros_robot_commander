#!/usr/bin/env python

import rospy
import math
import numpy as np

# homogeneous to cartesian coordinate transform
def hom_to_car_3d(hom_coordinate):
	if np.shape(hom_coordinate) != (4, 4):
		print "The input parameter is not a 3d homogeneous transformation matrix!"	
		return False
	else:
		hom_coordinate = hom_coordinate / float(hom_coordinate[3, 3])
		x_t = hom_coordinate[0, 3]
		y_t = hom_coordinate[1, 3]
		z_t = hom_coordinate[2, 3]
		alpha = math.atan( hom_coordinate[1, 0] / float(hom_coordinate[0, 0]))
		beta = math.atan( -hom_coordinate[2, 0] / np.sqrt( np.power(hom_coordinate[2, 1], 2)  + np.power(hom_coordinate[2, 2], 2)))
		gamma = math.atan( hom_coordinate[2, 1] / float(hom_coordinate[2, 2]))
		return np.array([x_t, y_t, z_t, gamma, beta, alpha])

# cartesian to homogeneous coordinate transform
def car_to_hom_3d(car_coordinate):
	if np.shape(car_coordinate) != (6,):
		print "The input parameter is not a 3d cartesian matrix containing the translation and euler angles!"	
		return False
	else:
		alpha = car_coordinate[5]
		beta = car_coordinate[4]
		gamma = car_coordinate[3]

		# building up the homogeneous transformation matrix elements:
		hom_00 = math.cos(alpha) * math.cos(beta)
		hom_01 = math.cos(alpha) * math.sin(beta) * math.sin(gamma) - math.sin(alpha) * math.cos(gamma)
		hom_02 = math.cos(alpha) * math.sin(beta) * math.cos(gamma) + math.sin(alpha) * math.sin(gamma)
		hom_10 = math.sin(alpha) * math.cos(beta)
		hom_11 = math.sin(alpha) * math.sin(beta) * math.sin(gamma) + math.cos(alpha) * math.cos(gamma)
		hom_12 = math.sin(alpha) * math.sin(beta) * math.cos(gamma) - math.cos(alpha) * math.sin(gamma)
		hom_20 = -math.sin(beta)
		hom_21 = math.cos(beta) * math.sin(gamma)
		hom_22 = math.cos(beta) * math.cos(gamma)
		hom_03 = car_coordinate[0]
		hom_13 = car_coordinate[1]
		hom_23 = car_coordinate[2]
		
		hom_coordinate = np.matrix([[hom_00, hom_01, hom_02, hom_03],
				           [hom_10, hom_11, hom_12, hom_13],
				           [hom_20, hom_21, hom_22, hom_23],
				           [0.0, 0.0, 0.0, 1.0]])
		return hom_coordinate
	
		
