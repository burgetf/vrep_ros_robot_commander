#!/usr/bin/env python

import rospy
import roslib
import math
import random
import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from sensor_msgs.msg import JointState
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visp_hand2eye_calibration.msg import TransformArray
from tf.transformations import quaternion_from_matrix
import tf
import tf.transformations as tr
from geometry_msgs.msg import Quaternion, Vector3, Transform
from visualization_msgs.msg import Marker
from visp_hand2eye_calibration.srv import compute_effector_camera_quick
import datetime as dt

class PathWalker:

    def __init__(self, name):
        self.name = name

        # the names of the base_link and end_link according to the urdf model
        self.base_link = "iiwa_base_link"
        self.end_link = "iiwa_adapter"

        # two variables to compute the time fragment between the two callbacks
        self.time_A = dt.datetime.now()
        self.time_B = dt.datetime.now()

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

        # initialize a node
        rospy.init_node("path_walker_node")

        self.robot = URDF.from_xml_file("/home/davoud/kuka_ws/src/kuka/kuka/iiwa/iiwa_description/urdf/iiwa.urdf")
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.kdl_kin = KDLKinematics(self.robot, self.base_link, self.end_link)

        # open the calibration_data file for saving the coordinates of end_effector and marker
        self.file = open("calibration_data", 'a')
        self.file.write("\n\nNew Run    Time: " + str(dt.datetime.now()) + "\n")

        # current position of the end effector
        # self.end_eff_current_pose = np.empty((1, 7))

        # number of poses for the path
        self.num_of_poses = 13

        # current pose of path
        self.current_pose = 0

        # the two transform arrays to call the service
        self.world_effector_transform_array = TransformArray()
        self.world_effector_transform_array.header.stamp = rospy.Time.now()
        self.world_effector_transform_array.header.frame_id = "iiwa_base_link"
        self.camera_object_transform_array = TransformArray()
        self.camera_object_transform_array.header.stamp = self.world_effector_transform_array.header.stamp
        self.camera_object_transform_array.header.frame_id = "camera_rgb_optical_frame"

        # list of poses
        """
        self.pose1 =  [-0.38, -1.79,  0.00,  0.00, -1.48, -1.90, -1.32]
        self.pose3 =  [-0.65, -1.51,  0.13,  0.00, -1.44, -1.75, -1.06]
        self.pose5 =  [-0.81, -1.33,  0.17, -0.37, -1.25, -1.71, -0.71]
        self.pose7 =  [-0.45, -1.33,  0.17, -0.79, -1.42, -1.76, -0.40]
        self.pose9 =  [-0.06, -0.90,  0.17, -1.28, -1.88, -1.76, -0.40]
        self.pose11 =  [-0.06, -0.20,  0.17, -1.28, -1.88, -1.76, -0.42]
        self.pose13 =  [-0.06,  0.27,  0.16, -1.06, -1.88, -1.76, -0.08]
        self.pose15 =  [-0.05,  1.25,  0.15, -0.43, -1.88, -1.76, -0.08]
        self.pose17 =  [-1.64,  1.21,  0.97, -1.80, -2.95, -1.57, -2.20]
        self.pose19 = [ 0.10,  1.09,  0.97, -0.19, -2.94, -1.57, -1.83]
        self.pose21 = [ 0.78,  1.10, -1.08, -0.19, -0.98, -1.33, -1.51]
        self.pose23 = [ 1.54,  1.34, -1.08, -0.19, -2.15, -0.66, -1.30]
        self.pose25 = [ 1.98,  1.13, -1.27, -0.68, -2.15, -0.66, -1.30]
        self.pose27 = [ 2.15,  1.93, -2.17, -0.48, -2.45, -1.06, -1.10]
        self.pose29 = [ 2.15,  1.63, -2.17, -0.48, -2.15, -0.86, -1.30]
        self.pose31 = [ 0.85, -0.23, -2.77,  1.88, -2.15,  0.46,  0.90]

        self.pose2 =   [-0.38, -1.79,  0.00,  0.00, -1.48, -1.70, -1.42]
        self.pose4 =   [-0.65, -1.51,  0.13,  0.00, -1.44, -1.55, -1.16]
        self.pose6 =   [-0.81, -1.33,  0.17, -0.37, -1.25, -1.51, -0.81]
        self.pose8 =   [-0.45, -1.33,  0.17, -0.79, -1.42, -1.56, -0.50]
        self.pose10 =  [-0.06, -0.90,  0.17, -1.28, -1.88, -1.56, -0.50]
        self.pose12 =  [-0.06, -0.20,  0.17, -1.28, -1.88, -1.56, -0.52]
        self.pose14 =  [-0.06,  0.27,  0.16, -1.06, -1.88, -1.56, -0.18]
        self.pose16 =  [-0.05,  1.25,  0.15, -0.43, -1.88, -1.56, -0.18]
        self.pose18 =  [-1.64,  1.21,  0.97, -1.80, -2.95, -1.37, -2.30]
        self.pose20 =  [ 0.10,  1.09,  0.97, -0.19, -2.94, -1.77, -1.93]
        self.pose22 =  [ 0.78,  1.10, -1.08, -0.19, -0.98, -1.53, -1.61]
        self.pose24 =  [ 1.54,  1.34, -1.08, -0.19, -2.15, -0.56, -1.40]
        self.pose26 =  [ 1.98,  1.13, -1.27, -0.68, -2.15, -0.56, -1.40]
        self.pose28 =  [ 2.15,  1.93, -2.17, -0.48, -2.45, -1.06, -1.20]
        self.pose30 =  [ 2.15,  1.63, -2.17, -0.48, -2.15, -0.86, -1.00]
        self.pose32 =  [ 0.85, -0.23, -2.77,  1.88, -2.15,  0.26,  0.80]
        """
        self.pose1 = [ 0.39, 1.67, -2.77, 0.07, -2.15, 0.26, 1.19]
        self.pose2 = [ 0.44, 1.67, -2.77, 0.07, -2.15, 0.26, 1.19]
        self.pose3 = [ 0.49, 1.67, -2.77, 0.07, -2.15, 0.26, 1.19]
        self.pose4 = [ 0.34, 1.67, -2.77, 0.07, -2.15, 0.26, 1.19]
        self.pose5 = [ 0.29, 1.67, -2.77, 0.07, -2.15, 0.26, 1.19]
        self.pose6 = [ 0.39, 1.72, -2.77, 0.07, -2.15, 0.26, 1.19]
        self.pose7 = [ 0.39, 1.77, -2.77, 0.07, -2.15, 0.26, 1.19]
        self.pose8 = [ 0.39, 1.62, -2.77, 0.07, -2.15, 0.26, 1.19]
        self.pose9 = [ 0.39, 1.57, -2.77, 0.07, -2.15, 0.26, 1.19]
        self.pose10 = [ 0.39, 1.57, -2.77, 0.07, -2.15, 0.26, 1.14]
        self.pose11 = [ 0.39, 1.57, -2.77, 0.07, -2.15, 0.26, 1.09]
        self.pose12 = [ 0.39, 1.57, -2.77, 0.07, -2.15, 0.26, 1.24]
        self.pose13 = [ 0.39, 1.57, -2.77, 0.07, -2.15, 0.26, 1.29]
        self.pose14 = [ 0.39, 1.57, -2.77, 0.07, -2.15, 0.26, 1.34]
        self.pose15 = [ 0.39, 1.57, -2.77, 0.07, -2.15, 0.26, 1.39]
        self.pose16 = [ 0.39, 1.57, -2.77, 0.07, -2.15, 0.26, 1.44]
        self.pose17 = [0.39, 1.57, -2.77, 0.07, -2.15, 0.26, 1.49]

        # transform listener
        self.transform_listener = tf.TransformListener()

        # the two current transforms
        self.world_effector_transform = Transform()
        self.camera_object_transform = Transform()

        # actionlib preparation
        self.manipulator_client = actionlib.SimpleActionClient("/iiwa/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.manipulator_client.wait_for_server()
        self.trajectory = JointTrajectory()
        self.trajectory.joint_names = self.joint_names
        self.trajectory.points.append(JointTrajectoryPoint())

        # subscribe to "/joint_states" topic which is published by kuka.
        rospy.Subscriber("/joint_states", JointState, self.kuka_inf_cb, queue_size=1)

        # subscribe to "/aruco_single/marker" which is published by aruco_ros
        # rospy.Subscriber("aruco_single/marker", Marker, self.marker_pose_cb)

        # subscribe to to "/avg_marker" which is computed by averaging two markers
        rospy.Subscriber("/avg_marker", Marker, self.marker_pose_cb)


    def kuka_inf_cb(self, data):
        end_eff_current_pose = data.position
        pose = self.kdl_kin.forward(end_eff_current_pose)
        pose_trans = pose[0:3,3]
        pose_rot = pose[0:3, 0:3]
        q_from_pose = quaternion_from_matrix(pose)

        vec3 = Vector3()
        vec3.x = pose_trans[0, 0]
        vec3.y = pose_trans[1, 0]
        vec3.z = pose_trans[2, 0]

        q = Quaternion()
        q.x = q_from_pose[0]
        q.y = q_from_pose[1]
        q.z = q_from_pose[2]
        q.w = q_from_pose[3]


        t = Transform()
        t.translation.x = vec3.x
        t.translation.y = vec3.y
        t.translation.z = vec3.z
        t.rotation.x = q.x
        t.rotation.y = q.y
        t.rotation.z = q.z
        t.rotation.w = q.w

        if not (math.isnan(vec3.x) or math.isnan(vec3.y) or math.isnan(vec3.z) or math.isnan(q.x) or math.isnan(q.y) or math.isnan(q.z) or math.isnan(q.w)):
            self.world_effector_transform = t
            self.time_B = dt.datetime.now()
            self.time_diff = self.time_B - self.time_A
            # print "The time difference between the callbacks is: %s" % str(self.time_diff)


    def marker_pose_cb(self, data):
        q = Quaternion()
        q.x = data.pose.orientation.x
        q.y = data.pose.orientation.y
        q.z = data.pose.orientation.z
        q.w = data.pose.orientation.w

        vec3 = Vector3()
        vec3.x = data.pose.position.x
        vec3.y = data.pose.position.y
        vec3.z = data.pose.position.z

        t = Transform()
        t.translation.x = vec3.x
        t.translation.y = vec3.y
        t.translation.z = vec3.z
        t.rotation.x = q.x
        t.rotation.y = q.y
        t.rotation.z = q.z
        t.rotation.w = q.w

        if not (math.isnan(vec3.x) or math.isnan(vec3.y) or math.isnan(vec3.z) or math.isnan(q.x) or math.isnan(q.y) or math.isnan(q.z) or math.isnan(q.w)):
            self.camera_object_transform = t
            self.time_A = dt.datetime.now()
        else:
            print "we have nan values!"


    def walk_the_path(self, pose_num):
        exec("%s = %s" % ('selected_pose', 'self.pose'+str(pose_num)))
        self.trajectory.points[0].positions = selected_pose
        self.trajectory.points[0].velocities = [0.0] * len(self.joint_names)
        self.trajectory.points[0].accelerations = [0.0] * len(self.joint_names)
        self.trajectory.points[0].time_from_start = rospy.Duration(1.0)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = self.trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.manipulator_client.send_goal(goal)
        self.manipulator_client.wait_for_result()

        self.manipulator_client.send_goal(goal)
        self.manipulator_client.wait_for_result()

    def calibration_coordinator(self):
        while self.current_pose < self.num_of_poses:
            self.current_pose += 1
            self.walk_the_path(self.current_pose)
            rospy.sleep(10.0)

            ############################################################################################################
            ### fixed values for error checking
            # fixed object_in_base T
            alpha, beta, gamma = 0.26, -1.72, 2.14
            xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
            Rx = tr.rotation_matrix(alpha, xaxis)
            Ry = tr.rotation_matrix(beta, yaxis)
            Rz = tr.rotation_matrix(gamma, zaxis)
            R = tr.concatenate_matrices(Rx, Ry, Rz)

            T1 = tr.translation_matrix([1.0, 1.0, 1.0])
            O_B_T = tr.concatenate_matrices(T1, R)
            # print "O_B_T: ", O_B_T

            # fixed cam_in_ee T
            alpha2, beta2, gamma2 = 0.0, 0.0, 3.14
            Rx2 = tr.rotation_matrix(alpha2, xaxis)
            Ry2 = tr.rotation_matrix(beta2, yaxis)
            Rz2 = tr.rotation_matrix(gamma2, zaxis)
            R2 = tr.concatenate_matrices(Rx2, Ry2, Rz2)
            T2 = tr.translation_matrix([0.0, 0.2, 0.0])
            C_E_T = tr.concatenate_matrices(T2, R2)
            # print "C_E_T: ", C_E_T

            # random ee_in_base T
            R_e_b = tr.random_rotation_matrix()
            T_e_b = tr.translation_matrix([random.random(), random.random(), random.random()])
            E_B_T = tr.concatenate_matrices(T_e_b, R_e_b)
            # print "E_B_T: ", E_B_T

            # calculated corresponding object_in_cam T
            O_C_T = np.dot(np.linalg.inv(np.dot(E_B_T, C_E_T)), O_B_T)
            # print "O_C_T:", O_C_T

            # value extraction for Transforms

            # world_effector transoform
            q_E_B_T = quaternion_from_matrix(E_B_T)
            t_E_B_T = Transform()
            t_E_B_T.translation.x =  E_B_T[0, 3]
            t_E_B_T.translation.y = E_B_T[1, 3]
            t_E_B_T.translation.z = E_B_T[2, 3]
            t_E_B_T.rotation.x = q_E_B_T[0]
            t_E_B_T.rotation.y = q_E_B_T[1]
            t_E_B_T.rotation.z = q_E_B_T[2]
            t_E_B_T.rotation.w = q_E_B_T[3]

            # camera_object transoform
            q_O_C_T = quaternion_from_matrix(O_C_T)
            t_O_C_T = Transform()
            t_O_C_T.translation.x =  O_C_T[0, 3]
            t_O_C_T.translation.y = O_C_T[1, 3]
            t_O_C_T.translation.z = O_C_T[2, 3]
            t_O_C_T.rotation.x = q_O_C_T[0]
            t_O_C_T.rotation.y = q_O_C_T[1]
            t_O_C_T.rotation.z = q_O_C_T[2]
            t_O_C_T.rotation.w = q_O_C_T[3]

            # self.world_effector_transform_array.transforms.append(t_E_B_T)
            # self.camera_object_transform_array.transforms.append(t_O_C_T)

            ############################################################################################################



            self.camera_object_transform_array.transforms.append(self.camera_object_transform)
            self.world_effector_transform_array.transforms.append(self.world_effector_transform)

            # write the data in "calibration_data.txt" file
            self.file.write("Step %s: \n" % str(self.current_pose))
            self.file.write("Camera_Object:\n%s\n" % str(self.camera_object_transform))
            # self.file.write("World_Effector:\n%s\n\n" % str(self.world_effector_transform))

            # print for checking
            print ("\nlenght of transform arrays are: %d , %d" % (len(self.world_effector_transform_array.transforms), len(self.camera_object_transform_array.transforms)))
            print "we are at the pose number: ", self.current_pose
            # print "added_transform for world_eff: ", self.world_effector_transform_array.transforms[self.current_pose-1]
            print "added_transform for cam_object: ", self.camera_object_transform_array.transforms[self.current_pose-1]

            # broadcast the tfs for checking in rviz
            w_e_tf = self.world_effector_transform_array.transforms[self.current_pose-1]
            c_o_tf = self.camera_object_transform_array.transforms[self.current_pose-1]


            ############################################################################################################
            # checkup
            # the object in camera frame:
            w_e_euler = tr.euler_from_quaternion([w_e_tf.rotation.x, w_e_tf.rotation.y, w_e_tf.rotation.z, w_e_tf.rotation.w], 'rxyz')
            xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
            Rx_we = tr.rotation_matrix(w_e_euler[0], xaxis)
            Ry_we = tr.rotation_matrix(w_e_euler[1], yaxis)
            Rz_we = tr.rotation_matrix(w_e_euler[2], zaxis)
            R_we = tr.concatenate_matrices(Rx_we, Ry_we, Rz_we)
            trans_we = tr.translation_matrix([w_e_tf.translation.x, w_e_tf.translation.y, w_e_tf.translation.z])
            w_e_t = tr.concatenate_matrices(trans_we, R_we)

            c_o_euler = tr.euler_from_quaternion([c_o_tf.rotation.x, c_o_tf.rotation.y, c_o_tf.rotation.z, c_o_tf.rotation.w], 'rxyz')
            Rx_co = tr.rotation_matrix(c_o_euler[0], xaxis)
            Ry_co = tr.rotation_matrix(c_o_euler[1], yaxis)
            Rz_co = tr.rotation_matrix(c_o_euler[2], zaxis)
            R_co = tr.concatenate_matrices(Rx_co, Ry_co, Rz_co)
            trans_co = tr.translation_matrix([c_o_tf.translation.x, c_o_tf.translation.y, c_o_tf.translation.z])
            c_o_t = tr.concatenate_matrices(trans_co, R_co)


            Rx_ce = tr.rotation_matrix(0.0, xaxis)
            Ry_ce = tr.rotation_matrix(0.0, yaxis)
            Rz_ce = tr.rotation_matrix(np.pi, zaxis)
            R_ce = tr.concatenate_matrices(Rx_ce, Ry_ce, Rz_ce)
            trans_ce = tr.translation_matrix([0.0, 0.22, 0.04])
            c_e_t = tr.concatenate_matrices(trans_ce, R_ce)

            # print "estimated object in base transform: \n", np.dot(w_e_t, np.dot(c_e_t, c_o_t))
            ############################################################################################################


            # the two tfs broadcasted to the rviz
            # w_e_tf and c_o_tf are the last Transforms added to the TransformArray
            br_eff = tf.TransformBroadcaster()
            br_eff.sendTransform((w_e_tf.translation.x, w_e_tf.translation.y, w_e_tf.translation.z),
                                (w_e_tf.rotation.x, w_e_tf.rotation.y, w_e_tf.rotation.z, w_e_tf.rotation.w),
                                rospy.Time.now(),
                                "computed_ee",
                            "iiwa_base_link")

            br_obj = tf.TransformBroadcaster()
            br_obj.sendTransform((c_o_tf.translation.x, c_o_tf.translation.y, c_o_tf.translation.z),
                    (c_o_tf.rotation.x, c_o_tf.rotation.y, c_o_tf.rotation.z, c_o_tf.rotation.w),
                        rospy.Time.now(),
                        "computed_obj",
                        "camera_rgb_optical_frame")


            # call the calibration service for checking its change through the path
            rospy.wait_for_service('compute_effector_camera_quick')
            try:
                quick_calibration_new = rospy.ServiceProxy('compute_effector_camera_quick', compute_effector_camera_quick)
                eff_cam_transform_new = quick_calibration_new(self.camera_object_transform_array, self.world_effector_transform_array)
                print "eff_cam_transform_for_check: \n", eff_cam_transform_new.effector_camera

            except rospy.ServiceException, e:
                print "Service call failed: %s" % e


        if self.current_pose is self.num_of_poses:
            print "the path is finished. Let's call the calibration service!\n"
            try:
                quick_calibration = rospy.ServiceProxy('compute_effector_camera_quick', compute_effector_camera_quick)
                eff_cam_transform = quick_calibration(self.camera_object_transform_array, self.world_effector_transform_array)
                # print "eff_cam_transform: \n", eff_cam_transform.effector_camera


				# conversion of the effector_camera Transform to transformation matrix "eff_optical_t"
                eff_optical_euler = tr.euler_from_quaternion([eff_cam_transform.effector_camera.rotation.x,
															  eff_cam_transform.effector_camera.rotation.y,
															  eff_cam_transform.effector_camera.rotation.z,
															  eff_cam_transform.effector_camera.rotation.w], 'rxyz')
                xaxis12, yaxis12, zaxis12 = [1, 0, 0], [0, 1, 0], [0, 0, 1]
                Rx12 = tr.rotation_matrix(eff_optical_euler[0], xaxis12)
                Ry12 = tr.rotation_matrix(eff_optical_euler[1], yaxis12)
                Rz12 = tr.rotation_matrix(eff_optical_euler[2], zaxis12)
                R12 = tr.concatenate_matrices(Rx12, Ry12, Rz12)
                trans12 = tr.translation_matrix([eff_cam_transform.effector_camera.translation.x,
												 eff_cam_transform.effector_camera.translation.y,
												 eff_cam_transform.effector_camera.translation.z])
                eff_optical_t = tr.concatenate_matrices(trans12, R12)


            	# extraction of Transform between camera_link and camera_rgb_optical_frame and conversion to transformation matrix
                (optical_to_cam_trans, optical_to_cam_rot) = self.transform_listener.lookupTransform('/camera_link',
                                                                                                     '/camera_rgb_optical_frame',
                                                                                                         rospy.Time(0))

                optical_to_cam_euler = tr.euler_from_quaternion([optical_to_cam_rot[0],
																 optical_to_cam_rot[1],
																 optical_to_cam_rot[2],
                         										 optical_to_cam_rot[3]], 'rxyz')
                xaxis1, yaxis1, zaxis1 = [1, 0, 0], [0, 1, 0], [0, 0, 1]
                Rx11 = tr.rotation_matrix(optical_to_cam_euler[0], xaxis1)
                Ry11 = tr.rotation_matrix(optical_to_cam_euler[1], yaxis1)
                Rz11 = tr.rotation_matrix(optical_to_cam_euler[2], zaxis1)
                R11 = tr.concatenate_matrices(Rx11, Ry11, Rz11)
                trans11 = tr.translation_matrix([optical_to_cam_trans[0],
												 optical_to_cam_trans[1],
												 optical_to_cam_trans[2]])
                optical_to_cam = tr.concatenate_matrices(trans11, R11)

                # print "eff_cam_opt_transform: \n", eff_optical_t
                # print "cam_link_to_opt_transform: \n", optical_to_cam

				# the final_tf is the eff to camera_link transform
                final_tf = np.dot (eff_optical_t, np.linalg.inv(optical_to_cam))
                # print "eff_cam_link_transform: \n", final_tf
                final_transform = Transform()
                final_transform.translation.x = final_tf[0, 3]
                final_transform.translation.y = final_tf[1, 3]
                final_transform.translation.z = final_tf[2, 3]
                final_quaternion = tr.quaternion_from_matrix(final_tf)
                final_transform.rotation.x = final_quaternion[0]
                final_transform.rotation.y = final_quaternion[1]
                final_transform.rotation.z = final_quaternion[2]
                final_transform.rotation.w = final_quaternion[3]

                print "eff_cam_link_transform vector: \n", final_transform

				# write the final answer in the file
                self.file.write("\n\nFinal Answer:\n%s\n\n" % eff_cam_transform)

				# broadcast the transform between camera_link and iiwa_adapter
                br_ans = tf.TransformBroadcaster()
                br_ans.sendTransform((final_transform.translation.x, final_transform.translation.y, final_transform.translation.z),
                            		(final_transform.rotation.x, final_transform.rotation.y, final_transform.rotation.z, final_transform.rotation.w),
                            		rospy.Time.now(),
                       	 			"camera_link",
                        			"iiwa_adapter")


                ########################################################################################################
                # checking how good is the answer!
                """
                for i in range(self.num_of_poses):
                    # world_effector
                    current_w_e_tf = self.world_effector_transform_array.transforms[i]
                    w_e_euler = tr.euler_from_quaternion([current_w_e_tf.rotation.x, current_w_e_tf.rotation.y, current_w_e_tf.rotation.z, current_w_e_tf.rotation.w], 'rxyz')
                    xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
                    Rx1 = tr.rotation_matrix(w_e_euler[0], xaxis)
                    Ry1 = tr.rotation_matrix(w_e_euler[1], yaxis)
                    Rz1 = tr.rotation_matrix(w_e_euler[2], zaxis)
                    R1 = tr.concatenate_matrices(Rx1, Ry1, Rz1)
                    trans1 = tr.translation_matrix([current_w_e_tf.translation.x, current_w_e_tf.translation.y, current_w_e_tf.translation.z])
                    w_e_t = tr.concatenate_matrices(trans1, R1)

                    # camera_object
                    current_c_o_tf = self.camera_object_transform_array.transforms[i]
                    c_o_euler = tr.euler_from_quaternion([current_c_o_tf.rotation.x, current_c_o_tf.rotation.y, current_c_o_tf.rotation.z, current_c_o_tf.rotation.w], 'rxyz')
                    Rx2 = tr.rotation_matrix(c_o_euler[0], xaxis)
                    Ry2 = tr.rotation_matrix(c_o_euler[1], yaxis)
                    Rz2 = tr.rotation_matrix(c_o_euler[2], zaxis)
                    R2 = tr.concatenate_matrices(Rx2, Ry2, Rz2)
                    trans2 = tr.translation_matrix([current_c_o_tf.translation.x, current_c_o_tf.translation.y, current_c_o_tf.translation.z])
                    c_o_t = tr.concatenate_matrices(trans2, R2)

                    # effector_camera
                    current_c_e_tf = eff_cam_transform.effector_camera
                    c_e_euler = tr.euler_from_quaternion([current_c_e_tf.rotation.x, current_c_e_tf.rotation.y, current_c_e_tf.rotation.z, current_c_e_tf.rotation.w], 'rxyz')
                    Rx3 = tr.rotation_matrix(c_e_euler[0], xaxis)
                    Ry3 = tr.rotation_matrix(c_e_euler[1], yaxis)
                    Rz3 = tr.rotation_matrix(c_e_euler[2], zaxis)
                    R3 = tr.concatenate_matrices(Rx3, Ry3, Rz3)
                    trans3 = tr.translation_matrix([current_c_e_tf.translation.x, current_c_e_tf.translation.y, current_c_e_tf.translation.z])
                    c_e_t = tr.concatenate_matrices(trans3, R3)

                    object_base_T_computed = np.dot(w_e_t, np.dot(c_e_t, c_o_t))
                    # print "********************"
                    # print "translation: ", object_base_T_computed[0:3, 3]
                    # print "rotation: ", tr.euler_from_matrix(object_base_T_computed, 'rxyz')
                """
                ########################################################################################################

            except rospy.ServiceException, e:
                print "Service call failed: %s" % e


my_path_walker = PathWalker("first")
my_path_walker.calibration_coordinator()


