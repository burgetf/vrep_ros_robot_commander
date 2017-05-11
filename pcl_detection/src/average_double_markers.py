#!/usr/bin/env python

import rospy
import roslib
import math
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix
import tf
import tf.transformations as tr
from geometry_msgs.msg import Quaternion, Vector3, Transform
import message_filters
from message_filters import TimeSynchronizer, Subscriber


class MarkerAverageCalc:
    def __init__(self, name):
        self.name = name
        self.pose1 = Pose()
        self.pose2 = Pose()

        # initialize a node
        rospy.init_node("marker_average_calc_node")

        self.avg_marker = Marker()
        self.avg_marker.header.frame_id = "camera_rgb_optical_frame"
        self.avg_marker.header.stamp = rospy.Time.now()
        self.avg_marker.type = Marker.SPHERE
        self.avg_marker.scale.x = 0.1
        self.avg_marker.scale.y = 0.1
        self.avg_marker.scale.z = 0.1
        self.avg_marker.color.a = 1.0
        self.avg_marker.color.r = 1.0
        self.avg_marker.color.g = 0.0
        self.avg_marker.color.b = 0.0

        # publisher of the averaged marker
        self.avg_marker_pub = rospy.Publisher("/avg_marker", Marker, queue_size=10)

        # subscribe to "/aruco_simple/pose" topic which is published by aruco_ros.
        rospy.Subscriber("/aruco_simple/pose", Pose, self.first_pose_cb, queue_size=1)
        # subscribe to "/aruco_simple/pose2" topic which is published by aruco_ros
        rospy.Subscriber("aruco_simple/pose2", Pose, self.second_pose_cb, queue_size=1)

        rospy.spin()

    def first_pose_cb(self, data):
        # save the first pose
        self.pose1.position.x = data.position.x
        self.pose1.position.y = data.position.y
        self.pose1.position.z = data.position.z
        self.pose1.orientation.x = data.orientation.x
        self.pose1.orientation.y = data.orientation.y
        self.pose1.orientation.z = data.orientation.z
        self.pose1.orientation.w = data.orientation.w

    def second_pose_cb(self, data):
        # save the second pose
        self.pose2.position.x = data.position.x
        self.pose2.position.y = data.position.y
        self.pose2.position.z = data.position.z
        self.pose2.orientation.x = data.orientation.x
        self.pose2.orientation.y = data.orientation.y
        self.pose2.orientation.z = data.orientation.z
        self.pose2.orientation.w = data.orientation.w

        self.avg_marker_computation()

    def avg_marker_computation(self):

        # position part
        self.avg_marker.pose.position.x = (self.pose1.position.x + self.pose2.position.x) / 2.0
        self.avg_marker.pose.position.y = (self.pose1.position.y + self.pose2.position.y) / 2.0
        self.avg_marker.pose.position.z = (self.pose1.position.z + self.pose2.position.z) / 2.0

        p1_q = self.pose1.orientation
        p2_q = self.pose2.orientation

        Q = np.array([[p1_q.x, p2_q.x],
                      [p1_q.y, p2_q.y],
                      [p1_q.z, p2_q.z],
                      [p1_q.w, p2_q.w]])
        Q_T = np.transpose(Q)
        mult = np.dot(Q, Q_T)
        (eigen_val, eigen_vec) = np.linalg.eig(mult)
        max_index = list(eigen_val).index(np.max(eigen_val))
        avg_q = eigen_vec[:, max_index]

        # orientation part
        self.avg_marker.pose.orientation.x = avg_q[0]
        self.avg_marker.pose.orientation.y = avg_q[1]
        self.avg_marker.pose.orientation.z = avg_q[2]
        self.avg_marker.pose.orientation.w = avg_q[3]

        avg_computed_marker = tf.TransformBroadcaster()
        avg_computed_marker.sendTransform((self.avg_marker.pose.position.x, self.avg_marker.pose.position.y, self.avg_marker.pose.position.z),
                             (self.avg_marker.pose.orientation.x, self.avg_marker.pose.orientation.y, self.avg_marker.pose.orientation.z, self.avg_marker.pose.orientation.w),
                             rospy.Time.now(),
                             "computed_avg_marker",
                             "camera_rgb_optical_frame")

        # other parts
        self.avg_marker_pub.publish(self.avg_marker)


my_calibrator_synchronizer = MarkerAverageCalc("first")
