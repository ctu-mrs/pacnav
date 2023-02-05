#include <ros/ros.h>
#include <ros/package.h>
#include <functional>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <memory>
#include <mrs_lib/transformer.h>

// Parameters: uav_name, uvdar_invalidate_time, uvdar_uav_names, uvdar_topics

class UvdarListener {
  using callback_t = boost::function<void(const nav_msgs::Odometry &)>;

  std::vector<ros::Subscriber> subscribers;
  std::vector<callback_t> callbacks;
  std::vector<std::shared_ptr<nav_msgs::Odometry>> states;
  std::mutex mutex;

  std::shared_ptr<mrs_lib::Transformer> transformer;
  std::string required_frame_id;
  double invalidate_time;

  bool is_initialized = false;

public:
  UvdarListener() = default;
  UvdarListener(std::shared_ptr<mrs_lib::Transformer> transformer, std::string required_frame_id);
  std::vector<Eigen::Vector3d> getEigenPositions();
};

