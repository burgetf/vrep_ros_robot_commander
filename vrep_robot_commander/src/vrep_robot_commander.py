#!/usr/bin/env python


import rospy
import numpy as np
import time
from vrep_common.msg import VrepInfo, JointSetStateData
from std_msgs.msg import Int32MultiArray, UInt8MultiArray, Float32MultiArray
from vrep_common.srv import * 

class RobotCommander:
	def __init__(self, name):
		# initialize the needed variables of the class
		self.name = name
		self.num_of_joints = 7
		self.motor_handles = [None] * self.num_of_joints
		self.joint_control_modes = [None] * self.num_of_joints
		self.simulation_running = True
		self.simulation_time = 0
		self.motor_handles_available = False

		# initialize the ros node for the class
		rospy.init_node(name, anonymous=True)

		# subscribe to the "/vrep/info" topic which is available even before simulation starts.
		rospy.Subscriber("/vrep/info", VrepInfo, self.vrep_info_callback)

		# subscribe to the "/vrep_ros_communication/motor_handles" topic which is availabe when the simulation is running.
		rospy.Subscriber("/vrep_ros_communication/motor_handles", Int32MultiArray, self.motor_handles_callback)

		# initialize the publisher of the manipulator joint speeds
		# JointSetStateData.msg has 3 data values: handles, setModes, values
		self.lbr_motor_speed_pub = rospy.Publisher("/vrep_ros_communication/lbr_joints", JointSetStateData, queue_size=10)

		# service client to get current joint positions of manipulator from Vrep
		# simRosGetJointState.srv takes "handle" and returns "result" and "state"
		rospy.wait_for_service("/vrep/simRosGetJointState")
		self.joint_config_client = rospy.ServiceProxy("/vrep/simRosGetJointState", simRosGetJointState)
		

		# service client to start the simulation mode in vrep
		# simRosStartSimulation takes nothing and returns "result"
		print_data_var = False
		rospy.wait_for_service("/vrep/simRosStartSimulation")
		start_simulation_client = rospy.ServiceProxy("/vrep/simRosStartSimulation", simRosStartSimulation)
		start_simulation_result = start_simulation_client()

		if print_data_var:
			rospy.loginfo("The result of start_simulation service is %s. ", start_simulation_result)

		# wait until the motor handles are available.
		while not self.motor_handles_available:
			rospy.loginfo("Waiting for the motor handles!\n Please activate the simulation in vrep.")
			# time.sleep(5)
			rospy.sleep(5)
		
		rospy.loginfo("The motor handles are available now!")

		# set the joint control modes
		# it can be in either velocity mode (0) or position mode (1)
		self.set_initial_control_modes(0)
		
		# print the joint states:		
		print_joint_state_var = False
		if print_joint_state_var:
			for i in range(self.num_of_joints):
				joint_config_resp = joint_config_client(self.motor_handles[i])
				print(joint_config_resp)

			

	# VrepInfo.msg has 4 data values: headerInfo, simulatorState, simulationTime, timeStep
	def vrep_info_callback(self, data):
		# if you want to check the data vlues of the VrepInfo message enable the print_data_var.
		print_data_var = False
		
		# update the class variables according to the values of the topic:
		self.simulation_running = bool(data.simulatorState.data)
		self.simulation_time = data.simulationTime.data

		# print the complete data of the topic
		if print_data_var:
			rospy.loginfo('The value of headerInfo is  %s.' , data.headerInfo)
			rospy.loginfo('The value of simulatorState is  %s.' , data.simulatorState.data)
			rospy.loginfo('The value of simulationTime is  %s.' , data.simulationTime.data)

			rospy.loginfo('The value of timeStep  %s.' , data.timeStep.data)

	# Int32MultiArray has 2 data values : layout, data
	def motor_handles_callback(self, data):
		# if you want to check the data vlues of the VrepInfo message enable the print_data_var.
		print_data_var = False

		# update the class variables according to the values of the topic:
		self.motor_handles = data.data
		self.motor_handles_available = True

		# print the complete data of the topic
		if print_data_var:
			rospy.loginfo('The value of layout is  %s.' , data.layout)
			rospy.loginfo('The value of data is  %s.' , data.data)
		rospy.spin()

	# set initial control modes for the manipulator joints.
	# it can be in velocity mode (0) or position mode (1)
	def set_initial_control_modes(self, chosen_mode):
		for i in range(len(self.joint_control_modes)):
			self.joint_control_modes[i] = chosen_mode
		print("initial_control_modes")
		self.set_control_modes()

	
	# set joint control modes for the manipulator joints.
	# simRosSetObjectIntParameter.srv takes "handle", "parameter", "parameterValue and returns "reslult".
	def set_control_modes(self):
		modes_set_var = True
		sim_jointintparam_ctrl_enabled = 2001
		
		for i in range(len(self.motor_handles)):
			# rospy.wait_for_service("/vrep/simRosSetObjectIntParameter")
			set_control_modes_client = rospy.ServiceProxy("/vrep/simRosSetObjectIntParameter", 
										simRosSetObjectIntParameter)
			# set the control mode for a joint
			modes_set_var = set_control_modes_client(self.motor_handles[i], sim_jointintparam_ctrl_enabled, 												self.joint_control_modes[i])
		print("control_modes")
		return modes_set_var

	
	# set the initial robot pose
	# simRosSetRobotPose.srv takes "handle", "relativeToObjectHandle", "position" and returns "result".
	def set_initial_robot_pose(self, robot_pose = None):
		print("started")
		rospy.wait_for_service("/vrep/simRosSetInitialRobotPose")
		set_initial_robot_pose_client = rospy.ServiceProxy("/vrep/simRosSetInitialRobotPose", simRosSetRobotPose)
		if robot_pose == None:
			robot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		set_initial_robot_pose_result = set_initial_robot_pose_client(robot_pose)
		print("initial_robot_pose_result")
		rospy.spin()

	# compute and execute joint velocities
	# we use the "self.lbr_motor_speed_pub" publisher initialized in class __init__.
	# JointSetStateData.msg has 3 data values: handles, setModes, values
	# For example handles' type is Int32MultiArray which has two attributes "data" and "layout" 
	# setModes wil set the set Joint Dynamic Properties in V-Rep and it takes either of values: 1, 2 or 3
        #       1: sets the target position (when joint is dynamically enabled and in position control)
        #	2:  sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
        # 	3: sets the maximum force/torque that the joint can exert
	def set_manipulator_joint_speed(self, joint_velocities):
		joint_speed_msg = JointSetStateData()
		joint_speed_msg.handles.data = self.motor_handles
		joint_speed_msg.setModes.data = [2, 2, 2, 2, 2, 2, 2]
		joint_speed_msg.values.data = joint_velocities

		self.lbr_motor_speed_pub.publish(joint_speed_msg)
		print("start moving the manipulator according to the velocity!")

	# compute and execute joint angle positions
	# we use the "self.lbr_motor_speed_pub" publisher initialized in class __init__.
	# JointSetStateData.msg has 3 data values: handles, setModes, values
	# For example handles' type is Int32MultiArray which has two attributes "data" and "layout" 
	# setModes wil set the set Joint Dynamic Properties in V-Rep and it takes either of values: 1, 2 or 3
        #       1: sets the target position (when joint is dynamically enabled and in position control)
        #	2:  sets the target velocity (when joint is dynamically enabled without position control, or when joint is in velocity mode)
        # 	3: sets the maximum force/torque that the joint can exert
	def set_manipulator_joint_position(self, joint_positions):
		joint_position_msg = JointSetStateData()
		joint_position_msg.handles.data = self.motor_handles
		joint_position_msg.setModes.data = [1, 1, 1, 1, 1, 1, 1]
		joint_position_msg.values.data = joint_positions

		self.lbr_motor_speed_pub.publish(joint_position_msg)
		print("start moving the manipulator according to the position!")


	# stop the manipulator's motion
	def stop_manipulator_motion(self):
		joint_speed_msg = JointSetStateData()
		joint_speed_msg.handles.data = self.motor_handles
		joint_speed_msg.setModes.data = [2, 2, 2, 2, 2, 2, 2]
		joint_speed_msg.values.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

		self.lbr_motor_speed_pub.publish(joint_speed_msg)
		print("manipulator stoped!")
		rospy.spin()


	# stop the manipulator's joint motion
	def stop_joint_motion(self, joint_num):
		joint_speed_msg = JointSetStateData()
		joint_speed_msg.handles.data = [self.motor_handles[joint_num]]
		joint_speed_msg.setModes.data = [2]
		joint_speed_msg.values.data = [0.0]

		self.lbr_motor_speed_pub.publish(joint_speed_msg)
		print("joint %d stoped!"% joint_num)
		rospy.spin()
	
	
	# print the joint states:
	def print_joint_states(self):
		print("joint information printing started!")		
		for i in range(self.num_of_joints):
			joint_config_resp = self.joint_config_client(self.motor_handles[i])
			print(joint_config_resp.state.name, ":  position is ", joint_config_resp.state.position, " velocity is ", 					joint_config_resp.state.velocity)	
		print("joint information printing ended!")

	# get the joint velocities:
	def get_joint_velocities(self):
		current_joint_velocities = np.array([0.0] * self.num_of_joints)
		for i in range(self.num_of_joints):
			joint_config_resp = self.joint_config_client(self.motor_handles[i])
			current_joint_velocities[i] = joint_config_resp.state.velocity[0]

		print "the current joint velocities are: ", current_joint_velocities
		return current_joint_velocities

	# get the joint positions:
	def get_joint_positions(self):
		current_joint_positions = np.array([0.0] * self.num_of_joints)
		for i in range(self.num_of_joints):
			joint_config_resp = self.joint_config_client(self.motor_handles[i])
			current_joint_positions[i] = joint_config_resp.state.position[0]

		# print "the current joint positions are: ", current_joint_positions
		return current_joint_positions




		


		

		
