#pragma once
#ifndef SHARED_GPS_AGGR_H
#define SHARED_GPS_AGGR_H

/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <ros/master.h>

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

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/PoseWithCovarianceIdentified.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

/* for operations with matrices */
#include <eigen3/Eigen/Eigen>

#include <swarm_utils/IdState.h>
#include <swarm_utils/IdStateArrayStamped.h>

#include <swarm_utils/ros_utils.h>

//}

using namespace std;
namespace e = Eigen;

namespace swarm_utils {

  namespace nodes {

    /* class GpsData//{ */
    class GpsData {
      private:
        double invalidate_time_ = 0;
        mutex mutex_data_;
        std::shared_ptr<nav_msgs::Odometry> data_;

      public:
        string topic;

        GpsData(double invalidate_time, string topic_name);
        void callbackGps(const nav_msgs::OdometryPtr msg);
        std::shared_ptr<nav_msgs::Odometry> getData();
    };
    //}

    /* class SharedGpsAggr //{ */
    class SharedGpsAggr : public nodelet::Nodelet {

      public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

      private:
        ros::NodeHandle nh_;

        /* flags */
        bool is_initialized_ = false;

        /* ros parameters //{ */

        string _uav_name_ = "uav2";
        string _origin_frame_                 = "local_origin";
        string _gps_topic_name_ = "odom_gps";

        double _invalidate_time_                      = 1.0;
        double _timer_rate_                = 1;

        //}

        /* common variables */
        std::shared_ptr<mrs_lib::Transformer> transformer_;
        vector<ros::Subscriber> sub_aggr_topics_;
        mutex mutex_gps_data_;
        vector<GpsData*> gps_data_;

        // | --------------------- timer callbacks -------------------- |

        void           callbackTimerSharedGpsAggr(const ros::TimerEvent& te);
        ros::Timer     timer_aggr_;
        ros::Publisher pub_shared_gps_aggr_;


        void callbackTimerDiscoverTopics([[maybe_unused]] const ros::TimerEvent& te);
        ros::Timer     timer_discvr_topics_;

        // | --------------------- routines -------------------- |

        vector<string> findNewTopics(string topic_name, vector<GpsData*>& gps_data);
    };
    //}

  }
}  // namespace swarm_util_nodes

#endif
