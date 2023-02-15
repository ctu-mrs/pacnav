#include "swarm_utils/ros_utils.h"

namespace swarm_utils {

  namespace ros_utils {

    std_msgs::Header createHeader(std::string frame_id, ros::Time stamp, uint32_t seq) {
      std_msgs::Header header;
      header.frame_id = frame_id;
      header.stamp = stamp;
      header.seq = seq;
      return header;
    }

    geometry_msgs::Vector3 createVector3(double x, double y, double z) {
      geometry_msgs::Vector3 vector3;
      vector3.x = x;
      vector3.y = y;
      vector3.z = z;
      return vector3;
    }

    geometry_msgs::Vector3Stamped createVector3Stamped(const std_msgs::Header &header, const geometry_msgs::Vector3 &vector) {
      geometry_msgs::Vector3Stamped vector3Stamped;
      vector3Stamped.header = header;
      vector3Stamped.vector = vector;
      return vector3Stamped;
    }

    geometry_msgs::Point createPoint(double x, double y, double z) {
      geometry_msgs::Point point;
      point.x = x;
      point.y = y;
      point.z = z;
      return point;
    }

    geometry_msgs::PointStamped createPointStamped(const std_msgs::Header &header, const geometry_msgs::Point &point) {
      geometry_msgs::PointStamped pointStamped;
      pointStamped.header = header;
      pointStamped.point = point;
      return pointStamped;
    }

    geometry_msgs::Pose createPose(const geometry_msgs::Point &position, const geometry_msgs::Quaternion &orientation) {
      geometry_msgs::Pose pose;
      pose.position = position;
      pose.orientation = orientation;
      return pose;
    }

    geometry_msgs::PoseStamped createPoseStamped(const std_msgs::Header& header, const geometry_msgs::Pose &pose) {
      geometry_msgs::PoseStamped poseStamped;
      poseStamped.header = header;
      poseStamped.pose = pose;
      return poseStamped;
    }

    mrs_msgs::Reference createReference(const geometry_msgs::Point &position, const double heading) {
      mrs_msgs::Reference reference;
      reference.position = position;
      reference.heading = heading;
      return reference;
    }

    mrs_msgs::ReferenceStamped createReferenceStamped(const std_msgs::Header& header, const mrs_msgs::Reference &reference) {
      mrs_msgs::ReferenceStamped referenceStamped;
      referenceStamped.header = header;
      referenceStamped.reference = reference;
      return referenceStamped;
    }

    Eigen::Vector3d vector3ToEigen(const geometry_msgs::Vector3 &vector3) {
      return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
    }

    geometry_msgs::Vector3 vector3FromEigen(const Eigen::Vector3d& vec) {
      return createVector3(vec(0), vec(1), vec(2));
    }

    Eigen::Vector3d pointToEigen(const geometry_msgs::Point& point) {
      return Eigen::Vector3d(point.x, point.y, point.z);
    };

    geometry_msgs::Point pointFromEigen(const Eigen::Vector3d& vec) {
      return createPoint(vec(0), vec(1), vec(2));
    };

  }
}
