#!/usr/bin/env python

import rospy
import roslib
import numpy as np
from vrep_common.msg import DetectedObjInf
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# manipulator joint names
joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

# the focal length of the camera
f = 573.5

# error in image plane
err = None

# the image jacobian
img_J = None

# object distance from camera
distance = None

object_found = None

# error gain
K = 1

det_obj_data = None
current_pose = None

# you can use one of these methods:
# 1: Jacobian Transpose Method (JTM)
# 2: Jacobian Pseudoinverse Method (JPM)
# 3: Damped Least Squares Method (DLSM)
used_method = "DLSM"

# damping_factor for DLS Method
landa = 0.1

def kuka_inf_callback(data):
	global current_pose

	current_pose = data.position 

def obj_inf_callback(data):

	global err
	global img_J
	global f
	global object_found
	global K
	global pose
	global det_obj_data
	global distance

	if data.isValid.data == 1:
		det_obj_data = data

		# the image dimension is 450 * 600
		# error of the object pose from the image center	
		err = np.matrix([[data.u.data - 300], [data.v.data - 225]]) * K
		# print "error:", err

		# image Jacobian (2 * 6)
		u = data.u.data
		v = data.v.data
		Z = data.dist.data
		distance = Z
		# print ("distance", Z)
		img_J = np.matrix([[f/Z,   0,    -u/Z,   -(u*v)/f,              ((f**2) + (u**2))/f,    -v],
			               [0,     f/Z,  -v/Z,   -((f**2) + (u**2))/f,  (u*v)/f,                u]])
		
		object_found = True 
	else:
		print "The object detection result is not valid!"
		object_found = False
		
	
# the names of the base_link and end_link according to the urdf model
base_link = "iiwa_0_link"
end_link = "iiwa_adapter"

# initialize a node
rospy.init_node("manipulator_mover")

robot = URDF.from_xml_file("/home/davoud/kuka_ws/src/kuka/kuka/iiwa/iiwa_description/urdf/iiwa.urdf")
tree = kdl_tree_from_urdf_model(robot)
kdl_kin = KDLKinematics(robot, base_link, end_link)

# initialize a publisher for publishing the needed data for object pose saving in a file
# file_data_pub = rospy.Publisher("/vrep_ros_communication/file_data", FileStorageInf, queue_size=1)

# actionlib commands
manipulator_client = actionlib.SimpleActionClient("/iiwa/follow_joint_trajectory", FollowJointTrajectoryAction)
manipulator_client.wait_for_server()
trajectory = JointTrajectory()
trajectory.joint_names = joint_names
trajectory.points.append(JointTrajectoryPoint())

# subscribe to "/joint_states" topic which is published by kuka.
rospy.Subscriber("/joint_states", JointState, kuka_inf_callback)

# subscribe to "vrep/rgb/image" topic which is published by vrep kinect.
rospy.Subscriber("object_detection/obj_inf", DetectedObjInf, obj_inf_callback)


while not rospy.is_shutdown():	

	
	if object_found == True:
		
		# publish data for file savings
		"""
		file_data_msg = FileStorageInf()
		file_data_msg.isValid.data = det_obj_data.isValid.data
		file_data_msg.u.data = det_obj_data.u.data
		file_data_msg.v.data = det_obj_data.v.data
		file_data_msg.dist.data = det_obj_data.dist.data
		file_data_msg.pixelSize.data = det_obj_data.pixelSize.data
		file_data_msg.jointValues.data = q
		file_data_pub.publish(file_data_msg)
		"""

		# the inverse of image Jacobian
		img_J_sudo_inverse = np.linalg.pinv(img_J)

		# the manipulator Jacobian
		J = kdl_kin.jacobian(current_pose)

		if used_method == "DLSM":
			# use of damped_least_squares as matrix inverse
			J_inverse = np.dot(J.T,np.linalg.inv(np.dot(J, J.T) + ((landa ** 2) * np.identity(6))))

		elif used_method == "JPM":
			# use of matrix pseudo_inverse as matrix inverse
			J_inverse = np.linalg.pinv(J)

		elif used_method == "JTM":
			# use of matrix transpose as matrix inverse
			J_inverse = np.transpose(J) 


		print ("error in image plane: ", err)
		cam_err = np.dot(img_J_sudo_inverse, err)
		# print ("error in camera: ", cam_err)
		cam_ee_jacobian = np.array([[1,  0,  0,  0,  0,  0],
							   		[0, 1,  0,  0,  0,  0],
						       		[0,  0,  1,  0,  0,  0],
							   		[0,  0,  0,  1,  0,  0],
							   		[0,  0,  0,  0, 1,  0],
							   		[0,  0,  0,  0,  0,  1]])

		ee_err = np.dot(cam_ee_jacobian, cam_err)
		# print ("error in end_effector: ", ee_err)
		rospy.sleep(1)
		result = np.dot(J_inverse, ee_err)
		### 

		# the final inverse jacobian
		# final_J = np.dot(J_inverse, img_J_sudo_inverse)

		# the dereived final speed command for the manipulator
		# result = np.dot(final_J, err)
		speed_cmd = list(np.array(result).reshape(-1,))
		
		# block to handle speed publisher using time step integration
		t = 0.5
		pose_change = [speed * t for speed in speed_cmd]
		final_pose = [a+b for a, b in zip(current_pose, pose_change)]
		
		# actionlib commands
		trajectory.points[0].positions = final_pose
		trajectory.points[0].velocities = [0.0] * len(joint_names)
		trajectory.points[0].accelerations = [0.0] * len(joint_names)
		trajectory.points[0].time_from_start = rospy.Duration(1.0)

		print "The manipulator is moving..."

		goal = FollowJointTrajectoryGoal()
		goal.trajectory = trajectory
		goal.goal_time_tolerance = rospy.Duration(0.0)

		manipulator_client.send_goal(goal)
		manipulator_client.wait_for_result()

	elif object_found == False:

		# actionlib commands
		trajectory.points[0].positions = current_pose
		trajectory.points[0].velocities = [0.0] * len(joint_names)
		trajectory.points[0].accelerations = [0.0] * len(joint_names)
		trajectory.points[0].time_from_start = rospy.Duration(1.0)

		print "looking for the object ..."

		goal = FollowJointTrajectoryGoal()
		goal.trajectory = trajectory
		goal.goal_time_tolerance = rospy.Duration(0.0)

		manipulator_client.send_goal(goal)
		manipulator_client.wait_for_result()
						







