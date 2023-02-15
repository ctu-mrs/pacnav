/* include header file of this class */
#include "nodes/shared_gps_aggr.h"

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace swarm_utils {

  namespace nodes {

    /* class GpsData//{ */
    GpsData::GpsData(double invalidate_time, string topic_name){

      invalidate_time_ = invalidate_time;
      topic = topic_name;
    }

    void GpsData::callbackGps(const nav_msgs::OdometryPtr msg) {

      scoped_lock lck(mutex_data_);
      data_ = std::make_shared<nav_msgs::Odometry>(*msg);
    }

    std::shared_ptr<nav_msgs::Odometry> GpsData::getData(){

      scoped_lock lck(mutex_data_);

      /* delete the data if its invalid */
      if(data_) {

        if((ros::Time::now().toSec() - data_->header.stamp.toSec()) > invalidate_time_) {
          ROS_WARN_STREAM("[SharedGpsAggr] " << topic << " data is older than " << invalidate_time_ << "s, deleting");
          data_.reset();
        }
      }
      else { 
        ROS_WARN_STREAM_THROTTLE(2, "[SharedGpsAggr] No GPS data received for " << topic);
      }

      return data_;
    }
    //}

    /* class SharedGpsAggr()//{ */

    /* onInit() //{ */

    void SharedGpsAggr::onInit() {

      // | ---------------- set my booleans to false ---------------- |
      // but remember, always set them to their default value in the header file
      // because, when you add new one later, you might forger to come back here

      /* obtain node handle */
      nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

      /* waits for the ROS to publish clock */
      ros::Time::waitForValid();

      // | ------------------- load ros parameters ------------------ |
      //
      /* (mrs_lib implementation checks whether the parameter was loaded or not) */
      mrs_lib::ParamLoader pl(nh_, "SharedGpsAggr");

      pl.loadParam("uav_name", _uav_name_);
      pl.loadParam("origin_frame", _origin_frame_);
      pl.loadParam("gps_topic_name", _gps_topic_name_);

      pl.loadParam("timer_rate", _timer_rate_);
      pl.loadParam("invalidate_time", _invalidate_time_);

      /* Check if all parameters were loaded successfully */ 
      if (!pl.loadedSuccessfully())
      {
        /* If not, alert the user and shut the node down */ 
        ROS_WARN("[SharedGpsAggr] parameter loading failure");
        ros::shutdown();
      }

      // | ---------- Tf --------- |

      transformer_ = std::make_shared<mrs_lib::Transformer>(std::string("SharedGpsAggr"), _uav_name_);

      // | ------------------ initialize publishers ----------------- |
      pub_shared_gps_aggr_ = nh_.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("aggr_poses", 1);

      // | -------------------- initialize timers ------------------- |

      timer_aggr_ = nh_.createTimer(ros::Rate(_timer_rate_), &SharedGpsAggr::callbackTimerSharedGpsAggr, this);
      timer_discvr_topics_ = nh_.createTimer(ros::Rate(_timer_rate_/10), &SharedGpsAggr::callbackTimerDiscoverTopics, this);
      // you can disable autostarting of the timer by the last argument

      // | ---------- initialize dynamic reconfigure server --------- |
      /* reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh)); */
      /* ReconfigureServer::CallbackType f = boost::bind(&SharedGpsAggr::callbackDynamicReconfigure, this, _1, _2); */
      /* reconfigure_server_->setCallback(f); */

      ROS_INFO_ONCE("[SharedGpsAggr] initialized");

      is_initialized_ = true;
    }
    //}

    // | --------------------- timer callbacks -------------------- |

    /* callbackTimerDiscoverTopics() //{ */

    void SharedGpsAggr::callbackTimerDiscoverTopics([[maybe_unused]] const ros::TimerEvent& te) {

      if (!is_initialized_)
        return;

      vector<string> topics = findNewTopics(_gps_topic_name_, gps_data_);

      scoped_lock lck(mutex_gps_data_);

      for(int i = 0; i < topics.size(); ++i){
        gps_data_.push_back(new swarm_utils::nodes::GpsData(_invalidate_time_, topics[i]));
        sub_aggr_topics_.push_back(nh_.subscribe(topics[i], 1, &swarm_utils::nodes::GpsData::callbackGps, gps_data_.back(), ros::TransportHints().tcpNoDelay()));
        ROS_INFO_STREAM("[SharedGpsAggr] Subscribing to topic: " << topics[i]);
      }

      if (sub_aggr_topics_.size() == 0) {
        ROS_WARN_STREAM("[SharedGpsAggr] Topic " << _gps_topic_name_ << " does not exist");
      }
    }

    //}

    /* callbackTimerSharedGpsAggr() //{ */

    void SharedGpsAggr::callbackTimerSharedGpsAggr([[maybe_unused]] const ros::TimerEvent& te) {

      if (!is_initialized_)
        return;

      mrs_msgs::PoseWithCovarianceArrayStamped msg_aggr_gps_data;
      msg_aggr_gps_data.header.stamp = ros::Time::now();
      msg_aggr_gps_data.header.frame_id = _uav_name_ + "/" + _origin_frame_;


      /* only publish the data that is not invalid. Identity is preserved as the data is always in the */
      /* sequence in which it was registered initially */
      if(gps_data_.size() != 0) {

        for(size_t i = 0; i < gps_data_.size(); i++) {
          auto data = gps_data_[i]->getData();

          if(data) {

            if(data->header.frame_id.find(_origin_frame_) != std::string::npos) {

              mrs_msgs::PoseWithCovarianceIdentified tmp;
              tmp.id = i;
              tmp.pose = data->pose.pose;

              msg_aggr_gps_data.poses.push_back(tmp);
            }
            else {
              ROS_ERROR("[SharedGpsAggr] Received GPS data not in '%s' frame", _origin_frame_.c_str());
            }
          }
        }

        ROS_INFO_THROTTLE(5, "[SharedGpsAggr] Publishing aggregated data");
      }
      else { 
        ROS_WARN_THROTTLE(5, "[SharedGpsAggr] No topic subscribed");
      }

      pub_shared_gps_aggr_.publish(msg_aggr_gps_data);
    }

    //}

    // | --------------------- routines -------------------- |

    /* findNewTopics //{ */

    vector<string> SharedGpsAggr::findNewTopics(string topic_name, vector<GpsData*>& gps_data){
      vector<string> new_topics;
      ros::master::V_TopicInfo topic_infos;
      ros::master::getTopics(topic_infos);

      for(auto it = topic_infos.begin(); it != topic_infos.end(); it++) {

        if(it->name.find(topic_name) != std::string::npos && it->name.find(_uav_name_) == std::string::npos) {

          bool found = false;
          scoped_lock lck(mutex_gps_data_);

          for(int i = 0; i < gps_data.size(); ++i) {

            if(it->name == gps_data[i]->topic) {
              found = true;
              break;
            }
          }

          if(!found){
            ROS_INFO_STREAM("[SharedGpsAggr] Found topic with desired name: " << it->name);
            new_topics.push_back(it->name);
          }
        }
      }

      return new_topics;
    }

    //}

    //}

  }
}  // namespace swarm_util_nodes

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(swarm_utils::nodes::SharedGpsAggr, nodelet::Nodelet);
