#!/usr/bin/env python

# import required packages
import rospy
import numpy as np
import time
import sys
from vrepConst import *

# import needed messages and services:
from std_msgs.msg import Int32MultiArray
from vrep_common.msg import ProximitySensorData, VrepInfo, JointSetStateData
from vrep_common.srv import simRosSetObjectIntParameter, simRosSetRobotPose, simRosEnablePublisher, simRosEnableSubscriber


# some global variables:
simulation_running = True	
simulation_time = 0
num_of_joints = 7
joint_motor_handles = [None] * num_of_joints

# VrepInfo.msg has 4 data values: headerInfo, simulatorState, simulationTime, timeStep
def vrep_info_callback(data):
	# if you want to check the data vlues of the VrepInfo message enable the print_data_var.
	print_data_var = False
		
	# update the class variables according to the values of the topic:
	simulation_running = bool(data.simulatorState.data)
	simulation_time = data.simulationTime.data

	# print the complete data of the topic
	if print_data_var:
		rospy.loginfo('The value of headerInfo is  %s.' , data.headerInfo)
		rospy.loginfo('The value of simulatorState is  %s.' , data.simulatorState.data)
		rospy.loginfo('The value of simulationTime is  %s.' , data.simulationTime.data)

		rospy.loginfo('The value of timeStep  %s.' , data.timeStep.data)



# the motor handles are passed as the arguments of the executable in VREP
# here we have just considered the 7 handles of the manipulator joints
if len(sys.argv)==num_of_joints+1: 
	for i in range(num_of_joints):
		joint_motor_handles[i] = int(sys.argv[i+1])
else:
	print("The arguments are not received from VREP properly!")

# initialize the ros node responsible for vrep ros communication
rospy.init_node('vrep_ros_communication')
print(joint_motor_handles)
print("VREP_ROS_COMMUNICATION just started.")

# publish the motor handles of the manipulator joints
lbr_motor_handles_pub = rospy.Publisher("/vrep_ros_communication/motor_handles", Int32MultiArray, queue_size=1)
motor_handles_msg = Int32MultiArray()
motor_handles_msg.data = joint_motor_handles
lbr_motor_handles_pub.publish(motor_handles_msg)

# subscribe to the "/vrep/info" topic which is available even before simulation starts.
rospy.Subscriber("/vrep/info", VrepInfo, vrep_info_callback)

# tell V-REP to subscribe to the motor speed topic:
# simRosEnableSubscriber.srv takes "topicName", "queueSize", "streamCmd" and returns "subscriberID".
rospy.wait_for_service("/vrep/simRosEnableSubscriber")
enable_subscriber_client = rospy.ServiceProxy("/vrep/simRosEnableSubscriber", simRosEnableSubscriber)

simros_strmcmd_set_joint_state = 0x000803
req_topicname = "/vrep_ros_communication/lbr_joints"
req_queuesize = 1
req_streamcmd = simros_strmcmd_set_joint_state
enable_subscriber_lbr_joints_var = enable_subscriber_client(req_topicname, req_queuesize, req_streamcmd, 0, 0, '')
if enable_subscriber_lbr_joints_var != -1:
	print("subscribed to '/vrep_ros_communication/lbr_joints' properly!")

while not rospy.is_shutdown():
	print("motor handles are being published...")
	lbr_motor_handles_pub.publish(motor_handles_msg)
	rospy.sleep(5)


	
	


