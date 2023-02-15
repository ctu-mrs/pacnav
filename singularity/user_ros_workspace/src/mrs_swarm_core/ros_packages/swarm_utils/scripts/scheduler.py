#!/usr/bin/env python3
import math
import subprocess, shlex, psutil
import os
import sys
import time
import numpy as np

import rospy
import tf2_geometry_msgs
import tf2_ros
from nav_msgs.msg import Odometry
from std_msgs.msg import Time
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg
from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.srv import Vec4Request
from mrs_msgs.srv import Vec4


# global lists used for getting callback data
last_odom_ = []
failed_flight_ = []

# #{ odomCallback

def odomCallback(odom, uav_id):
    # rospy.loginfo('receiving odom for ' + str(uav_id))

    last_odom_[uav_id-1] = odom

# #} end of odomCallback

# #{ diagCallback

def diagCallback(msg, uav_id):
    # rospy.loginfo('receiving diag for ' + str(uav_id))

    failed_flight_[uav_id-1] = not msg.flying_normally

# #} end of diagCallback

# #{ tfPose

def tfPose(input_position, input_frame, des_frame):
    stamped_point = geometry_msgs.msg.PointStamped()
    stamped_point.point = input_position
    stamped_point.header.frame_id = input_frame
    stamped_point.header.stamp = rospy.Time(0)

    trans = tf_buffer_.lookup_transform(des_frame, input_frame, rospy.Time())
    tf_pose = tf2_geometry_msgs.do_transform_point(stamped_point, trans)

    return tf_pose

# #} end of tfPose

# #{ setGoal

def setGoal(l_id, goal, srv_name):

    client_setgoal = rospy.ServiceProxy('/uav'+str(l_id)+srv_name, Vec4)

    msg_goal = Vec4Request([goal.x, goal.y, goal.z, 0.0])
    try:
        resp = client_setgoal(msg_goal)
        if resp.success == True:
            return True
        else:
            return False

    except rospy.ServiceException as exc:
        return False

# #} end of setGoal

# #{ swarmAtGoal

def swarmAtGoal(odoms, goal, thresh_dist):

    g_vec = np.array([goal.x, goal.y])
    reached = 0

    for i in range(0, len(odoms)):
        tf_odom = tfPose(odoms[i].pose.pose.position, odoms[i].header.frame_id, 'global_origin')

        if np.linalg.norm(np.array([tf_odom.point.x, tf_odom.point.y]) - g_vec) < thresh_dist:
            reached += 1

    if reached == len(odoms):
        rospy.logwarn('All UAVs are inside dist: {}'.format(thresh_dist))
        return True
    else:
        return False

# #} end of 


if __name__ == '__main__':
    rospy.init_node('scheduler')
    # wait for node to start, otherwise rostime is zero
    rospy.sleep(2.0)

    # init time used to check max timeout
    init_time = rospy.get_rostime()
    # sys-time gives a unique name for the rosbag
    sys_time = time.gmtime()

    # episode count
    _ep_count_ = rospy.get_param('~episode_count')

    # num of uavs in the exp
    _uav_count_ = rospy.get_param('~uav_count')

    # threshold dist to check if the uavs have reached goal
    _inter_uav_dist_ = rospy.get_param('~inter_uav_dist') 
    # the formula is r >= sqrt(3 * N * uav_rad^2)
    goal_arrival_dist_ = math.sqrt(3 * _uav_count_ * _inter_uav_dist_**2)

    # total time for uavs to get ready
    _preflight_timeout_ = _uav_count_ * rospy.get_param('~preflight_timeout')

    # max time after which the exp does not make sense
    _max_timeout_ = rospy.get_param('~max_timeout')

    # num of leader with goal coordinates
    _leader_count_ = rospy.get_param('~leader_count')
    _leader_goal_x_ = rospy.get_param('~goal_x')
    _leader_goal_y_ = rospy.get_param('~goal_y')

    # service to set goal
    _setgoal_srv_ = rospy.get_param('~setgoal_srv')
    # service to start the control algorithm
    _runswarm_srv_ = rospy.get_param('~runswarm_srv')

    # dir name for bag files
    _bag_file_dir_ = rospy.get_param('~bag_file_dir')

    # listen to tf tree for transformations
    tf_buffer_ = tf2_ros.Buffer()
    tf_listener_ = tf2_ros.TransformListener(tf_buffer_)

    # stop sim signal is published as a trigger
    pub_stop_sim = rospy.Publisher('stop_sim', Time, queue_size=2)
    ros_rate = rospy.Rate(10)

    leader_goal = geometry_msgs.msg.Point()
    leader_goal.x = _leader_goal_x_
    leader_goal.y = _leader_goal_y_
    leader_goal.z = 0.0

    sub_diag_list_ = []
    sub_odom_list_ = []
    sub_lidar_list_ = []
    for uav_id in range(1, _uav_count_ + 1):
        last_odom_.append(None)
        failed_flight_.append(False)

        # diagnostics to check if the uavs were setup and took off properly
        sub_diag_list_.append(rospy.Subscriber('/uav'+str(uav_id)+'/control_manager/diagnostics', ControlManagerDiagnostics, callback=diagCallback, callback_args=uav_id, queue_size=1))

        # odometry to check of uavs have proper localization, also acts as double check for failure
        sub_odom_list_.append(rospy.Subscriber('/uav'+str(uav_id)+'/odometry/odom_main', Odometry, callback=odomCallback, callback_args=uav_id, queue_size=1))
        # sub_lidar_list_.append(rospy.Subscriber('/uav'+str(uav_id)+'/rplidar/scan', LaserScan, callback=laserCallback, callback_args=uav_id, queue_size=1))

    # select uavs to give goal position
    leader_ids = np.random.choice(np.arange(1, _uav_count_ + 1), size=_leader_count_, replace=False)

    # make a new dir for this experiment
    suffix = str(_ep_count_)
    os.popen('mkdir '+_bag_file_dir_+'/'+suffix)

    # setup rosbag recording
    for uav_id in range(1, _uav_count_+1):
        uav_name = 'uav' + str(uav_id)

        rosbag_command = 'rosbag record -O '+_bag_file_dir_+'/'+suffix+'/'+uav_name+'.bag /tf /tf_static /'+uav_name+'/global_pose /'+uav_name+'/odometry/odom_main /'+uav_name+'/mrs_uav_status/display_string /'+uav_name+'/rplidar/scan'

        # make a shell command and run in a different process
        rosbag_command = shlex.split(rosbag_command)
        subprocess.Popen(rosbag_command)

    goal_set = False
    flight_end = False
    flight_fail = False

    while not rospy.is_shutdown():
        rospy.loginfo_throttle(3, 'checking status')
        cur_time = rospy.get_rostime()

        if (cur_time.secs - init_time.secs) > _preflight_timeout_:
            
            if (cur_time.secs - init_time.secs) > _max_timeout_ or any(flag == True for flag in failed_flight_) or any(odom == None for odom in last_odom_):
                flight_fail = True

            if flight_fail:
                rospy.logerr('flight failed')

                # close the rosbag recording properly
                # for proc in psutil.process_iter():
                #     if "record" in proc.name():
                #         proc.send_signal(subprocess.signal.SIGINT)

                # # give time for processes to close rosbag
                # rospy.sleep(2.0 * _uav_count_)

                # publish trigger for ending experiment
                cur_time = rospy.get_rostime()
                pub_stop_sim.publish(cur_time)

            elif not goal_set:
                rospy.logwarn_throttle(3, 'setting goal...')

                goal_status = []
                for l_id in leader_ids:
                    tf_goal = tfPose(leader_goal, 'global_origin', 'uav'+str(l_id)+'/local_origin')
                    goal_status.append(setGoal(l_id, tf_goal.point, _setgoal_srv_))

                if all(s == True for s in goal_status):

                    for uav_id in range(1, _uav_count_+1):
                        client_run_swarm = rospy.ServiceProxy('/uav'+str(uav_id)+_runswarm_srv_, Trigger)

                        try:
                            resp = client_run_swarm()
                            goal_set = True

                        except rospy.ServiceException as exc:
                            rospy.logwarn_throttle(3, 'run_swarm failed')
                            goal_set = False
                else:
                    rospy.logwarn_throttle(3, 'goal service failed for some UAVs')

                if goal_set:
                    rospy.loginfo('goal set for all the UAVs')
                    # goal_set = True

            elif swarmAtGoal(last_odom_, leader_goal, goal_arrival_dist_):
                rospy.logwarn('Reached goal!')

                # close the rosbag recording properly
                for proc in psutil.process_iter():
                    if "record" in proc.name():
                        proc.send_signal(subprocess.signal.SIGINT)

                # give time for processes to close rosbag
                rospy.sleep(2.0 * _uav_count_)

                # publish trigger for ending experiment
                cur_time = rospy.get_rostime()
                pub_stop_sim.publish(cur_time)

        ros_rate.sleep()
