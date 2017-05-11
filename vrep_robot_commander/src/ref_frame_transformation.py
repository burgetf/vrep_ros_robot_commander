#!/usr/bin/env python

# import needed packages
import numpy as np
import math

# shortening the function names
sin = math.sin
cos = math.cos

# initialization of the needed variables
# d_x, d_y and d_z are the positional transformation parameters between X1 and X2
d_x = 0
d_y = 0
d_z = 0

# psi, phi and theta are the rotational parameters (respectively for x, y and z axises) between X1 and X2
psi = float(np.pi/2)
phi = 0.0
# theta = float(np.pi)
theta = 0.0

# the transformation matrix between X1 and X2
T = np.array([[cos(theta)*cos(phi), -sin(theta)*cos(psi)+cos(theta)*sin(phi)*sin(psi), sin(theta)*sin(psi)+cos(theta)*sin(phi)*cos(psi), d_x],
	      [sin(theta)*cos(phi), cos(theta)*cos(psi)+sin(theta)*sin(phi)*sin(psi), -cos(theta)*sin(psi)+sin(theta)*sin(phi)*cos(psi), d_y],
              [-sin(phi),           cos(phi)*sin(psi),                                 cos(phi)*cos(psi),                                d_z],
              [0,                   0,                                                 0,                                                1]])
print "The transformation matrix is: ", T
