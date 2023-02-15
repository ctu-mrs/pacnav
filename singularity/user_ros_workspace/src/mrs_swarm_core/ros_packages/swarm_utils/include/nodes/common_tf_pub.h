#pragma once
#ifndef COMMONTFPUB_H
#define COMMONTFPUB_H

/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* /1* this header file is created during compilation from python script dynparam.cfg *1/ */
/* #include <waypoint_flier/dynparamConfig.h> */

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for mathematicla operations */
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

/* custom msgs of MRS group */
#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/EulerAngles.h>
#include <mrs_msgs/TransformVector3Srv.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

/* for operations with matrices */
#include <eigen3/Eigen/Eigen>

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/image_encodings.h>

/* for tranforming coordintes frames */
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//}

namespace swarm_utils {

  namespace nodes {

  /* class CommonTfPub //{ */
  class CommonTfPub : public nodelet::Nodelet {

    public:
      /* onInit() is called when nodelet is launched (similar to main() in regular node) */
      virtual void onInit();

    private:
      /* flags */
      bool is_initialized_ = false;

      /* ros parameters //{ */

      std::string _uav_name_ = "uav2";
      std::string _uav_frame_id_                 = "uav2/fcu_untilted";
      std::string _origin_frame_id_                 = "uav2/local_origin";
      std::string _global_origin_frame_id_                = "global_origin";
      double _invalidate_time_                      = 1.0;
      double _tf_timer_rate_                = 1;
      double _pose_x_global_frame_ = 0;
      double _pose_y_global_frame_ = 0;
      double _pose_z_global_frame_ = 0;
      double _yaw_global_frame_ = 0;

      //}

      /* common variables */
      tf2_ros::Buffer tfBuffer;
      tf2_ros::TransformListener tfListener{tfBuffer};

      // | ---------------------- msg callbacks --------------------- |
      // | --------------------- timer callbacks -------------------- |

      void           callbackTimerCommonTfPub(const ros::TimerEvent& te);
      ros::Timer     timer_transformer_;
      // | ---------------- service server callbacks ---------------- |
      // | --------------------- service clients -------------------- |
      // | ------------------- dynamic reconfigure ------------------ |
      // | -------------------- support functions ------------------- |
      // | -------------------- compute functions ------------------- |
  };
  //}

  }
}  // namespace swarm_util_nodes

#endif
