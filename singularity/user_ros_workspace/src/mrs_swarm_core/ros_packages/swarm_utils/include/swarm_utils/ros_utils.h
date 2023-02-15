#pragma once
#ifndef ROS_UTILS_H
#define ROS_UTILS_H

#include <ros/ros.h>
#include <ros/package.h>

#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <mrs_msgs/ReferenceStamped.h>

namespace swarm_utils {

  namespace ros_utils {

    // Header

    std_msgs::Header createHeader(std::string frame_id, ros::Time stamp, uint32_t seq=0);

    // Vector3

    geometry_msgs::Vector3 createVector3(double x, double y, double z);

    geometry_msgs::Vector3Stamped createVector3Stamped(const std_msgs::Header &header, const geometry_msgs::Vector3 &vector3);

    // Point

    geometry_msgs::Point createPoint(double x, double y, double z);

    geometry_msgs::PointStamped createPointStamped(const std_msgs::Header &header, const geometry_msgs::Point &point);

    // Pose

    geometry_msgs::Pose createPose(const geometry_msgs::Point &position, const geometry_msgs::Quaternion &orientation);

    geometry_msgs::PoseStamped createPoseStamped(const std_msgs::Header& header, const geometry_msgs::Pose &pose);

    // Reference

    mrs_msgs::Reference createReference(const geometry_msgs::Point &position, const double heading);

    mrs_msgs::ReferenceStamped createReferenceStamped(const std_msgs::Header& header, const mrs_msgs::Reference &reference);

    // Eigen

    Eigen::Vector3d vector3ToEigen(const geometry_msgs::Vector3 &vector3);

    geometry_msgs::Vector3 vector3FromEigen(const Eigen::Vector3d& vec);

    Eigen::Vector3d pointToEigen(const geometry_msgs::Point& point);

    geometry_msgs::Point pointFromEigen(const Eigen::Vector3d& vec);

 }
}

#endif
