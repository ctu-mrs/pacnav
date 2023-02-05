#!/usr/bin/env python3

import rospy
import rosbag
import tf2_ros
import tf2_py
import tf2_geometry_msgs
import geometry_msgs.msg
import nav_msgs.msg
import mrs_msgs.msg

import pickle
import numpy as np
import os
import sys
import csv
import time
import math

def tfData(data, to_frame):
    trans = tfBuffer.lookup_transform(to_frame, data.header.frame_id, rospy.Time(), rospy.Duration(1))
    pose_transformed = tf2_geometry_msgs.do_transform_pose(data, trans)
    pub_tf_pose.publish(pose_transformed)

    rospy.loginfo_throttle(10, "Transformed data from: {:s} to {:s}".format(data.header.frame_id, pose_transformed.header.frame_id))

def callbackPose(data):
    rospy.loginfo_throttle(2, "Receiving data")
    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.header = data.header
    pose_stamped.pose = data.pose.pose

    tfData(pose_stamped, _to_frame_)

if __name__ == '__main__':

    rospy.init_node('transform_msgs', anonymous=True)

    _uav_name_ = rospy.get_param('~uav_name')
    _to_frame_ = rospy.get_param('~to_frame')
    _in_topic_ = rospy.get_param('~in_topic')
    _out_topic_ = rospy.get_param('~out_topic')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    sub_pose = rospy.Subscriber(_in_topic_, nav_msgs.msg.Odometry, callbackPose)
    # sub_pose = rospy.Subscriber(_in_topic_, mrs_msgs.msg.PositionCommand, callbackPose)
    pub_tf_pose = rospy.Publisher(_out_topic_, geometry_msgs.msg.PoseStamped, queue_size=10)
    rospy.spin()
