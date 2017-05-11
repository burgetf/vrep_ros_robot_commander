#!/usr/bin/env python

import rospy
import numpy as np
import operator
import os
from my_lib import hom_to_car_3d, car_to_hom_3d
from vrep_common.msg import FileStorageInf
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

last_data = None

# initialize the needed variables
file_address = '/home/davoud/catkin_ws/src/file_saving/src/object_poses_file.txt'

def file_data_callback(data):
	global last_data
	global file_address

	if last_data != data:
		last_data = data
		isValid = data.isValid.data
		u = data.u.data
		v = data.v.data
		Z = data.dist.data
		pixelSize = data.pixelSize.data
		q = data.jointValues.data 

		# homogeneous matrix of the pose of end effector wrt the base
		base_to_end = kdl_kin.forward(q)
		
		# homogeneous matrix of the camera pose wrt end effector
		end_to_cam = car_to_hom_3d([0.0279, 0.0296, 0.0268, -np.pi/2, 0, 0])

		# save the object pose wrt the manipulator base in the file		
		cam_to_obj = np.matrix([[(u - 300) * pixelSize], [(255 - v) * pixelSize], [Z], [1]])
		object_pose = np.dot(base_to_end, np.dot(end_to_cam, cam_to_obj))
		object_pose_list = str(list(np.array(object_pose[0:3]).reshape(-1,)))

		# open the file to save the object poses
		f = open(file_address, 'a')
		f.write(object_pose_list + '\n')
		print object_pose_list
		f.close()


# the names of the base_link and end_link according to the urdf model
base_link = "iiwa_link_0"
end_link = "iiwa_link_ee_kuka"

robot = URDF.from_xml_file("/home/davoud/catkin_ws/src/iiwa_stack-master/iiwa_descriptions/urdf/iiwa7.urdf")
tree = kdl_tree_from_urdf_model(robot)
kdl_kin = KDLKinematics(robot, base_link, end_link)

# initialize the ros node
rospy.init_node('object_pose_saver', anonymous=True)
			
while not rospy.is_shutdown():	
	# subscribe to the "/vrep_ros_communication/motor_handles" topic which is availabe when the simulation is running.
	rospy.Subscriber("/vrep_ros_communication/file_data", FileStorageInf, file_data_callback)

