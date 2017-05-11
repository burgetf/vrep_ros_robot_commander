#!/usr/bin/env python

import rospy
import roslib
import math
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from aruco_msgs.msg import ArucoMarker
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Quaternion, Vector3, Transform
from sdh2_hand.srv import SDHAction
import tf.transformations as tr
import tf

class Mission:

    def __init__(self, name):
        self.name = name

        self.timer = 0

        # the names of the base_link and end_link according to the urdf model
        self.base_link = "iiwa_0_link"
        self.end_link = "iiwa_adapter"

        # initialize a node
        rospy.init_node("mission_node")

        self.robot = URDF.from_xml_file("/home/davoud/kuka_ws/src/kuka/kuka/iiwa/iiwa_description/urdf/iiwa.urdf")
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.kdl_kin = KDLKinematics(self.robot, self.base_link, self.end_link)

        # you can use one of these methods:
        # 1: Jacobian Transpose Method (JTM)
        # 2: Jacobian Pseudoinverse Method (JPM)
        # 3: Damped Least Squares Method (DLSM)
        self.used_method = "DLSM"
        self.landa = 0.1

        # tf broadcaster
        self.br = tf.TransformBroadcaster()
		
        # current position of the end effector
        self.end_eff_current_pose = np.empty((1, 7))

        # transformation matrix between end_effector and camera
        self.end_cam_T = np.empty((4, 4))
        self.set_end_cam_T()

        # transformation matrix between end_effector and gripper
        self.end_grip_T = np.empty((4, 4))
        self.set_end_grip_T()

        # transformation matrix between gripper and target pose
        self.grip_target_T = np.empty((4, 4))
        self.set_grip_target_T()

        # transformation matrix between base and end effector
        self.base_end_T = np.empty((4, 4))

        # transformation matrix between base and target
        self.base_target_T = np.empty((4, 4))

        # needed variables for actionlib
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

        # actionlib commands
        self.manipulator_client = actionlib.SimpleActionClient("/iiwa/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.manipulator_client.wait_for_server()
        self.trajectory = JointTrajectory()
        self.trajectory.joint_names = self.joint_names
        self.trajectory.points.append(JointTrajectoryPoint())

        # path of end_effector
        self.ee_path = MarkerArray()

        # path of object
        self.object_path = MarkerArray()

	
        # initialize a publisher to publish the marker of detected objet to rviz
        self.marker_object_in_base_pub = rospy.Publisher("marker_object_in_base", Marker, queue_size=1)
        self.marker_object_in_cam_pub = rospy.Publisher("marker_object_in_cam", Marker, queue_size=1)
        self.marker_eff_path_pub = rospy.Publisher("marker_eff_path", MarkerArray, queue_size=1)
        self.marker_obj_path_pub = rospy.Publisher("marker_obj_path", MarkerArray, queue_size=1)

        # initialize the service client for the gripper
        rospy.wait_for_service("/sdh_action")
        v = rospy.ServiceProxy("/sdh_action", SDHAction)

        # subscribe to "/joint_states" topic which is published by kuka.
        rospy.Subscriber("/joint_states", JointState, self.kuka_inf_cb)

        # subscribe to "/cylinder_marker" topic which is published by cylinder detector
        rospy.Subscriber("/cylinder_marker", Marker, self.marker_inf_cb, queue_size=1)
        rospy.spin()

    def kuka_inf_cb(self, data):
        if data._connection_header["callerid"] != "/kuka_manager" :
            return
        self.end_eff_current_pose = data.position
        self.set_base_end_T()
		

    def marker_inf_cb(self, data):
        print "in marker callback"
        q = Quaternion()
        q = data.pose.orientation

        marker_orientation = np.array([q.x, q.y, q.z, q.w])
        euler_angles = tr.euler_from_quaternion(marker_orientation, 'rxyz')

        vec3 = Vector3()
        vec3.x = data.pose.position.x
        vec3.y = data.pose.position.y
        vec3.z = data.pose.position.z

        marker_pose = np.array([vec3.x, vec3.y, vec3.z, euler_angles[0], euler_angles[1], euler_angles[2]])
		
        for i in range(6):
            if np.isnan(marker_pose[i]):
                return


        cam_object_T = self.set_cam_object_T(marker_pose)

        cam_object_R = tr.identity_matrix()
        cam_object_R[0:3, 0:3] = cam_object_T[0:3, 0:3]
        cam_object_q = tr.quaternion_from_matrix(cam_object_R)
        self.br.sendTransform((cam_object_T[0, 3], cam_object_T[1, 3], cam_object_T[2, 3]),
                              cam_object_q	,
                              rospy.Time.now(),
                              "object_in_cam_frame",
                              "camera_rgb_optical_frame")

        # publishing the object marker to rviz for object_in_cam
        det_marker = Marker()
        det_marker.header.frame_id = "camera_rgb_optical_frame"
        det_marker.header.stamp = rospy.Time.now()
        det_marker.type = Marker.SPHERE
        det_marker.pose.position.x = cam_object_T[0, 3]
        det_marker.pose.position.y = cam_object_T[1, 3]
        det_marker.pose.position.z = cam_object_T[2, 3]
        det_marker.pose.orientation.x = 0
        det_marker.pose.orientation.y = 0
        det_marker.pose.orientation.z = 0
        det_marker.pose.orientation.w = 1
        det_marker.scale.x = 0.05
        det_marker.scale.y = 0.05
        det_marker.scale.z = 0.05
        det_marker.color.a = 1.0
        det_marker.color.r = 1.0
        det_marker.color.g = 0.0
        det_marker.color.b = 0.0
        self.marker_object_in_cam_pub.publish(det_marker)


        base_object_T = np.dot(np.dot(self.base_end_T, self.end_cam_T), cam_object_T)
        # print base_object_T

        final_err = self.error_computation(base_object_T, self.base_target_T)

		

        # target thresholds:
        trans_th = [0.05, 0.05, 0.05]
        rot_th = [15 * np.pi/180, 15 * np.pi/180, 15 * np.pi/180]

        print "timer: ", self.timer
		
        if self.timer > 5 :
            print "goal reached ............................"
            self.gripper_final_action()

		
        if np.abs(final_err[0]) < trans_th[0] and np.abs(final_err[1]) < trans_th[1] and np.abs(final_err[2]) < trans_th[2]:
            if np.abs(final_err[3]) < rot_th[0] and np.abs(final_err[4]) < rot_th[1] and np.abs(final_err[5]) < rot_th[2]:
                self.timer += 1
                return
            else:
                self.timer = 0
                print "error :", final_err
        else:
            self.timer = 0
            print "error :", final_err

        # use of error_gains:
        error_gains = np.reshape(np.array([ 1, 1, 1, 1, 1, 1]), (6, 1))
        final_err2 = np.multiply(final_err, error_gains)
		

        # error value type 2
        end_object_T = np.dot(self.end_cam_T, cam_object_T)
        end_target_T = np.dot(self.end_grip_T, self.grip_target_T)
        object_target_T = np.dot(end_object_T, np.linalg.inv(end_target_T))
        # print "error style 2: ", object_target_T

        # publishing the object marker to rviz
        det_marker = Marker()
        det_marker.header.frame_id = "iiwa_base_link"
        det_marker.header.stamp = rospy.Time.now()
        det_marker.type = Marker.SPHERE
        det_marker.pose.position.x = base_object_T[0, 3]
        det_marker.pose.position.y = base_object_T[1, 3]
        det_marker.pose.position.z = base_object_T[2, 3]
        det_marker.pose.orientation.x = 0
        det_marker.pose.orientation.y = 0
        det_marker.pose.orientation.z = 0
        det_marker.pose.orientation.w = 1
        det_marker.scale.x = 0.05
        det_marker.scale.y = 0.05
        det_marker.scale.z = 0.05
        det_marker.color.a = 1.0
        det_marker.color.r = 0.0
        det_marker.color.g = 0.0
        det_marker.color.b = 1.0
        self.marker_object_in_base_pub.publish(det_marker)
				
        # print "final error is : ", final_err
        # print "base_target_T: ", base_object_T

        # computation of jacobian of the manipulator
        J = self.kdl_kin.jacobian(self.end_eff_current_pose)

        if self.used_method == "DLSM":
            # use of damped_least_squares as matrix inverse
            J_inverse = np.dot(J.T,np.linalg.inv(np.dot(J, J.T) + ((self.landa ** 2) * np.identity(6))))

        elif self.used_method == "JPM":
            # use of matrix pseudo_inverse as matrix inverse
            J_inverse = np.linalg.pinv(J)

        elif self.used_method == "JTM":
            # use of matrix transpose as matrix inverse
            J_inverse = np.transpose(J)

        final_vel = np.dot(J_inverse, final_err2)

        # the dereived final speed command for the manipulator
        speed_cmd = list(np.array(final_vel).reshape(-1,))

        # avoid huge joint velocities:
        joint_vel_th = [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7]
        if speed_cmd[0] > joint_vel_th[0]:
            print "huge velocity for the first joint!"
            return
        elif speed_cmd[1] > joint_vel_th[1]:
            print "huge velocity for the second joint!"
            return
        elif speed_cmd[2] > joint_vel_th[2]:
            print "huge velocity for the third joint!"
            return
        elif speed_cmd[3] > joint_vel_th[3]:
            print "huge velocity for the fourth joint!"
            return
        elif speed_cmd[4] > joint_vel_th[4]:
            print "huge velocity for the fifth joint!"
            return
        elif speed_cmd[5] > joint_vel_th[5]:
            print "huge velocity for the sixth joint!"
            return
        elif speed_cmd[6] > joint_vel_th[6]:
            print "huge velocity for the seventh joint!"
            return
		
        # block to handle speed publisher using pose publisher
        intergration_time_step = 0.5
        pose_change = [speed * intergration_time_step for speed in speed_cmd]
        final_pose = [a+b for a, b in zip(self.end_eff_current_pose, pose_change)]

        # actionlib commands
        self.trajectory.points[0].positions = final_pose
        self.trajectory.points[0].velocities = [0.0] * len(self.joint_names)
        self.trajectory.points[0].accelerations = [0.0] * len(self.joint_names)
        self.trajectory.points[0].time_from_start = rospy.Duration(1.0)

        # print "pose_change", pose_change
        # print "The manipulator is moving..."

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = self.trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)
		
        self.manipulator_client.send_goal(goal)
        action_result = self.manipulator_client.wait_for_result()

        if action_result is True:
            # publishing the path of the object in "red" color
            final_object_position = base_object_T[0:3, 3]
            final_object_R = tr.identity_matrix()
            final_object_R[0:3, 0:3] = base_object_T[0:3, 0:3]
            final_object_quaternion = tr.quaternion_from_matrix(final_object_R)
            self.object_path.markers.append(Marker())
            self.object_path.markers[-1].header.frame_id = "iiwa_base_link"
            self.object_path.markers[-1].header.stamp = rospy.Time.now()
            self.object_path.markers[-1].id = len(self.object_path.markers)
            self.object_path.markers[-1].type = Marker.SPHERE
            self.object_path.markers[-1].pose.position.x = final_object_position[0]
            self.object_path.markers[-1].pose.position.y = final_object_position[1]
            self.object_path.markers[-1].pose.position.z = final_object_position[2]
            self.object_path.markers[-1].pose.orientation.x = final_object_quaternion[0]
            self.object_path.markers[-1].pose.orientation.y = final_object_quaternion[1]
            self.object_path.markers[-1].pose.orientation.z = final_object_quaternion[2]
            self.object_path.markers[-1].pose.orientation.w = final_object_quaternion[3]
            self.object_path.markers[-1].scale.x = 0.01
            self.object_path.markers[-1].scale.y = 0.01
            self.object_path.markers[-1].scale.z = 0.01
            self.object_path.markers[-1].color.a = 1.0
            self.object_path.markers[-1].color.r = 1.0
            self.object_path.markers[-1].color.g = 0.0
            self.object_path.markers[-1].color.b = 0.0
            self.marker_obj_path_pub.publish(self.object_path)


            # publishing path of the end effector in "green" color
            final_ee_position = self.base_end_T[0:3, 3]
            final_ee_R = tr.identity_matrix()
            final_ee_R[0:3, 0:3] = self.base_end_T[0:3, 0:3]
            final_ee_quaternion = tr.quaternion_from_matrix(final_ee_R)
            self.ee_path.markers.append(Marker())
            self.ee_path.markers[-1].header.frame_id = "iiwa_base_link"
            self.ee_path.markers[-1].header.stamp = rospy.Time.now()
            self.ee_path.markers[-1].id = len(self.ee_path.markers)
            self.ee_path.markers[-1].type = Marker.SPHERE
            self.ee_path.markers[-1].pose.position.x = final_ee_position[0]
            self.ee_path.markers[-1].pose.position.y = final_ee_position[1]
            self.ee_path.markers[-1].pose.position.z = final_ee_position[2]
            self.ee_path.markers[-1].pose.orientation.x = final_ee_quaternion[0]
            self.ee_path.markers[-1].pose.orientation.y = final_ee_quaternion[1]
            self.ee_path.markers[-1].pose.orientation.z = final_ee_quaternion[2]
            self.ee_path.markers[-1].pose.orientation.w = final_ee_quaternion[3]
            self.ee_path.markers[-1].scale.x = 0.01
            self.ee_path.markers[-1].scale.y = 0.01
            self.ee_path.markers[-1].scale.z = 0.01
            self.ee_path.markers[-1].color.a = 1.0
            self.ee_path.markers[-1].color.r = 0.0
            self.ee_path.markers[-1].color.g = 1.0
            self.ee_path.markers[-1].color.b = 0.0
            self.marker_eff_path_pub.publish(self.ee_path)
 


		
    def set_end_cam_T(self):
        # translation and 180 degree rotation around z axis ???
        """
        delta_x = 0.02
        delta_y = 0.215
        delta_z = 0.03
        self.end_cam_T = np.array([[-1, 0, 0, delta_x],
                                   [0, -1, 0, delta_y],
                                   [0, 0, 1, delta_z],
                                   [0, 0, 0, 1]])
        """
        delta_x = 0.03
        delta_y = 0.13
        delta_z = 0.04
        self.end_cam_T = np.array([[-1, 0, 0, delta_x],
                                   [0, -1, 0, delta_y],
                                   [0, 0, 1, delta_z],
                                   [0, 0, 0, 1]])

        # print "end_cam_T: ", self.end_cam_T

    def set_end_grip_T(self):
        delta_x = 0.0
        delta_y = 0.0
        delta_z = 0.7
        self.end_grip_T = np.array([[1, 0, 0, delta_x],
                                    [0, 1, 0, delta_y],
                                    [0, 0, 1, delta_z],
                                    [0, 0, 0, 1]])
        # print "end_grip_T: ", self.end_grip_T
        """
		o_eulers = tr.euler_from_matrix(self.end_grip_T, 'rxyz')
    		self.br.sendTransform((delta_x, delta_y, 0),
                     tf.transformations.quaternion_from_euler(o_eulers[0], o_eulers[1], o_eulers[2]),
                     rospy.Time.now(),
                     "gripper",
                     "iiwa_adapter")
		"""

    def set_grip_target_T(self):
        # translation and no rotation ???
        delta_x = 0.0
        delta_y = 0.0
        delta_z = 0.05
        self.grip_target_T = np.array([[1, 0, 0, delta_x],
                                       [0, 1, 0, delta_y],
                                       [0, 0, 1, delta_z],
                                       [0, 0, 0, 1]])
        # print "grip_target_T: ", self.grip_target_T

    def set_base_end_T(self):
        pose = self.kdl_kin.forward(self.end_eff_current_pose)
        self.base_end_T = pose
		
        # print "base_end_T: ", self.base_end_T
        self.base_target_T = np.dot(np.dot(self.base_end_T, self.end_grip_T), self.grip_target_T)
        # print "base_target_T: ", self.base_target_T

    def set_cam_object_T(self, object_pose):
        Tr = tr.translation_matrix([object_pose[0], object_pose[1], object_pose[2]])
		
        xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
        Rx = tr.rotation_matrix(object_pose[3], xaxis)
        Ry = tr.rotation_matrix(object_pose[4], yaxis)
        Rz = tr.rotation_matrix(object_pose[5], zaxis)
        R = tr.concatenate_matrices(Rx, Ry, Rz)

        T = tr.concatenate_matrices(Tr, R)
        return T

    def gripper_final_action(self):
        self.timer = 0
        print "in gripper_final_action... "
        service_result = self.joint_config_client(0, 0, 0.0, 1.0)
        # print "sevice ", service_result
        pose_difference = np.array([[1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, -0.22],
                                    [0, 0, 0, 1]])
		
        final_pose_T = np.dot(self.base_target_T, pose_difference)


        # block of interpolation between current_pose and final_pose of ee

        print "final_pose_T : ", final_pose_T
		
        final_q_ik = self.kdl_kin.inverse(final_pose_T, np.array(self.end_eff_current_pose)+0.3)
        # print "final_q_ik : ", np.shape(final_q_ik), type(final_q_ik)
        if final_q_ik is None:
            print "The target could not be reached!"
            return
		
        # actionlib commands
        self.trajectory.points[0].positions = final_q_ik
        self.trajectory.points[0].velocities = [0.0] * len(self.joint_names)
        self.trajectory.points[0].accelerations = [0.0] * len(self.joint_names)
        self.trajectory.points[0].time_from_start = rospy.Duration(1.0)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = self.trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)
		
        self.manipulator_client.send_goal(goal)
        self.manipulator_client.wait_for_result()

        service_result = self.joint_config_client(0, 0, 1.0, 1.0)

    def lerp(p0, p1, t):
        return (1 - t) * p0 + t * p1


    def error_computation(self, base_object, base_target):
        position_err = base_object[0:3, 3] - base_target[0:3, 3]

        # computation of orientation_err using the skew symmetric matrix
        # the order for the result quaternion from tr package is x, y, z, w
        # no need to change the order in goal_q and ee_q
        cos = np.cos
        sin = np.sin
        base_object_R = tr.identity_matrix()
        base_object_R[0:3, 0:3] = base_object[0:3, 0:3]
        modification_matrix1 = np.array([[cos(4 * np.pi/6),	   0,	     sin(4 * np.pi/6),	 0],
                                         [0,                   1,        0,              0],
                                         [-sin(4 * np.pi/6),   0,        cos(4 * np.pi/6),   0],
                                         [0,                   0,        0,              1]])
        modification_matrix2 = np.array([[1,	          0,	         0,	           0],
                                         [0 ,             cos(np.pi),   -sin(np.pi),   0],
                                         [0,              sin(np.pi),    cos(np.pi),   0],
                                         [0,              0,             0,            1]])

        modified_base_object_R = np.dot(np.dot(base_object_R, modification_matrix1), modification_matrix2)
        base_object_quaternion = tr.quaternion_from_matrix(modified_base_object_R)
        goal_q = [base_object_quaternion[0], base_object_quaternion[1], base_object_quaternion[2], base_object_quaternion[3]]

		
        ### object tf wrt base before modification in orientation
        real_base_object_quaternion = tr.quaternion_from_matrix(base_object_R)
        object_real_q = [real_base_object_quaternion[0], real_base_object_quaternion[1], real_base_object_quaternion[2], real_base_object_quaternion[3]]
        self.br.sendTransform((base_object[0, 3], base_object[1, 3], base_object[2, 3]),
                              object_real_q,
                              rospy.Time.now(),
                              "actual_object_tf",
                              "iiwa_base_link")

        ### object tf wrt base after modification in orientation
        self.br.sendTransform((base_object[0, 3], base_object[1, 3], base_object[2, 3]),
                              goal_q	,
                              rospy.Time.now(),
                              "modified_object_tf",
                              "iiwa_base_link")
	
		
        # computation of orientation error using skey_symmetric matrix
        base_target_R = tr.identity_matrix()
        base_target_R[0:3, 0:3] = base_target[0:3, 0:3]
        base_target_quaternion = tr.quaternion_from_matrix(base_target_R)
        ee_q = [base_target_quaternion[0], base_target_quaternion[1], base_target_quaternion[2], base_target_quaternion[3]]

        skew_mat = np.array([[0.0,	  -goal_q[2],	goal_q[1]],
                             [goal_q[2],   0.0,		-goal_q[0]],
                             [-goal_q[1],  goal_q[0],    0.0]])

        orientation_err = np.empty((3, 1))
        for i in range(3):
            orientation_err[i] = (ee_q[3] * goal_q[i]) - (goal_q[3] * ee_q[i]) - (skew_mat[i, 0] * ee_q[0] + skew_mat[i, 1] * ee_q[1] + skew_mat[i, 2] * ee_q[2])



        # print "orientation error: ", orientation_err
			
        final_err = np.empty((6, 1))
        final_err[0] = position_err[0]
        final_err[1] = position_err[1]
        final_err[2] = position_err[2]
        # final_err[0] = 0
        # final_err[1] = 0
        # final_err[2] = 0
        final_err[3] = orientation_err[0]
        final_err[4] = orientation_err[1]
        final_err[5] = orientation_err[2]
        # final_err[3] = 0
        # final_err[4] = 0
        # final_err[5] = 0
        return final_err

mission1 = Mission("mission1")


