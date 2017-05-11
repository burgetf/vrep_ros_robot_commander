#!/usr/bin/env python


import rospy
import actionlib
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
pose1 = [0.018 , -0.707, -0.168, -1.313, 0.427, 1.335, 1.150]
pose2 = [0.518 , -0.707, -0.168, -1.313, 0.427, 1.335, 1.150]
pose3 = [1.4, 0.5, -0.2, -0.2, 0.2, 1.57, 0.0]
pose4 = [1.4, 0.5, -0.2, -0.5, 0.2, 1.0, 0.0]
pose5 = [1.7, 0.8, -0.2, -0.5, 0.2, 1.0, 0.5]
pose6 = [1.6, 1.0, -0.2, -0.5, 0.2, 0.3, 1.0]
pose7 = [ 0.39, 1.57, -2.77, 0.07, -2.15, 0.26, 1.34]

rospy.init_node("manipulator_mover")

manipulator_client = actionlib.SimpleActionClient("/iiwa/follow_joint_trajectory", FollowJointTrajectoryAction)
manipulator_client.wait_for_server()
trajectory = JointTrajectory()
trajectory.joint_names = joint_names

trajectory.points.append(JointTrajectoryPoint())
trajectory.points[0].positions = pose1
trajectory.points[0].velocities = [0.0] * len(joint_names)
trajectory.points[0].accelerations = [0.0] * len(joint_names)
trajectory.points[0].time_from_start = rospy.Duration(1.0)

"""
trajectory.points.append(JointTrajectoryPoint())
trajectory.points[1].positions = pose2
trajectory.points[1].velocities = [0.0] * len(joint_names)
trajectory.points[1].accelerations = [0.0] * len(joint_names)
trajectory.points[1].time_from_start = rospy.Duration(2.0)

trajectory.points.append(JointTrajectoryPoint())
trajectory.points[2].positions = pose3
trajectory.points[2].velocities = [0.0] * len(joint_names)
trajectory.points[2].accelerations = [0.0] * len(joint_names)
trajectory.points[2].time_from_start = rospy.Duration(5.0)

trajectory.points.append(JointTrajectoryPoint())
trajectory.points[3].positions = pose4
trajectory.points[3].velocities = [0.0] * len(joint_names)
trajectory.points[3].accelerations = [0.0] * len(joint_names)
trajectory.points[3].time_from_start = rospy.Duration(2.0)

trajectory.points.append(JointTrajectoryPoint())
trajectory.points[4].positions = pose5
trajectory.points[4].velocities = [0.0] * len(joint_names)
trajectory.points[4].accelerations = [0.0] * len(joint_names)
trajectory.points[4].time_from_start = rospy.Duration(5.0)
"""	
goal = FollowJointTrajectoryGoal()
goal.trajectory = trajectory
goal.goal_time_tolerance = rospy.Duration(0.0)

manipulator_client.send_goal(goal)
manipulator_client.wait_for_result()
		
	






