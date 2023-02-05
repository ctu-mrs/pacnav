#include "uvdar_listener.h"
#include <mrs_lib/param_loader.h>

/* UvdarListener() //{ */

UvdarListener::UvdarListener(std::shared_ptr<mrs_lib::Transformer> transformer, std::string required_frame_id) 
  :transformer(transformer), required_frame_id(required_frame_id)
{
  ros::NodeHandle nh = ros::NodeHandle("~");

  std::string uav_name;
  std::vector<std::string> uvdar_uav_names;
  std::vector<std::string> uvdar_topics;

  mrs_lib::ParamLoader paramLoader(nh, "UvdarListener");
  paramLoader.loadParam("uav_name", uav_name);
  paramLoader.loadParam("uvdar_uav_names", uvdar_uav_names);
  paramLoader.loadParam("uvdar_topics", uvdar_topics);
  paramLoader.loadParam("uvdar_invalidate_time", invalidate_time);

  if (!paramLoader.loadedSuccessfully()) {
    ROS_ERROR_STREAM("[UvdarListener] Failed to load some of the parameters!");
    ros::shutdown();
  }

  if (uvdar_uav_names.size() != uvdar_topics.size()) {
    ROS_ERROR_STREAM("[UvdarListener] Params uvdar_uav_names and uvdar_topics should have corresponding sizes!");
    ros::shutdown();
  }
  
  auto uav_it = std::find(uvdar_uav_names.begin(), uvdar_uav_names.end(), uav_name);
  if (uav_it == uvdar_uav_names.end()) {
    ROS_ERROR_STREAM("[UvdarListener] Own uav name " << uav_name << " cannot be found in the uvdar_uav_names. Not removing anything");
  } else {
    uvdar_uav_names.erase(uav_it);
    uvdar_topics.erase(uvdar_topics.begin() + (uav_it - uvdar_uav_names.begin()));
  }

  // Create the states to store data
  states.resize(uvdar_topics.size());

  // Create the callbacks
  for (size_t i = 0; i < uvdar_topics.size(); ++i) {
    auto &currentState = states[i];

    callbacks.emplace_back([&currentState, this] (const nav_msgs::Odometry &msg) {
        currentState.reset(new nav_msgs::Odometry(msg));
    });
  }

  // Create the subscirbers
  for (size_t i = 0; i < uvdar_topics.size(); ++i) {
    subscribers.push_back(
        nh.subscribe<nav_msgs::Odometry>(uvdar_topics[i], 1, callbacks[i])
    );
  }

  is_initialized = true;
}

//}

/* getEigenPositions() //{ */

std::vector<Eigen::Vector3d> UvdarListener::getEigenPositions() {
  std::vector<Eigen::Vector3d> positions;

  if (!is_initialized) {
    return positions;
  }


  for (size_t i = 0; i < states.size(); ++i) {
    geometry_msgs::PoseStamped pose;

    // Get the state
    std::unique_lock lock(mutex);
    auto odometry = states[i];
    if (!odometry) continue;
    pose.pose = odometry->pose.pose;
    pose.header = odometry->header;
    lock.unlock();

    // Transform
    std::optional<geometry_msgs::PoseStamped> result = 
      this->transformer->transformSingle(pose, this->required_frame_id);

    // Store in the vector
    if (result) {
      if (ros::Time::now().toSec() - pose.header.stamp.toSec() > invalidate_time)
        continue;
      geometry_msgs::Point point = result->pose.position;
      positions.emplace_back(point.x, point.y, point.z);
    } else {
      ROS_ERROR_STREAM("[UvdarListener] Cannot transform from uvdar frame " << 
          pose.header.frame_id << " to required frame " << this->required_frame_id);
    }
  }

  return positions;
}

//}

