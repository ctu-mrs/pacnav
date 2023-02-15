/* include header file of this class */
#include "nodes/common_tf_pub.h"

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace swarm_utils {

  namespace nodes {

    /* onInit() //{ */

    void CommonTfPub::onInit() {

      // | ---------------- set my booleans to false ---------------- |
      // but remember, always set them to their default value in the header file
      // because, when you add new one later, you might forger to come back here

      /* obtain node handle */
      ros::NodeHandle nh("~");

      /* waits for the ROS to publish clock */
      ros::Time::waitForValid();


      // | ------------------- load ros parameters ------------------ |
      //
      /* (mrs_lib implementation checks whether the parameter was loaded or not) */
      mrs_lib::ParamLoader pl(nh, "CommonTfPub");

      pl.loadParam("uav_name", _uav_name_);
      pl.loadParam("uav_frame_id", _uav_frame_id_);
      pl.loadParam("origin_frame_id", _origin_frame_id_);
      pl.loadParam("global_origin_frame_id", _global_origin_frame_id_);

      pl.loadParam("tf_timer_rate", _tf_timer_rate_);
      pl.loadParam("invalidate_time", _invalidate_time_);
      pl.loadParam(_uav_name_ + "/x", _pose_x_global_frame_);
      pl.loadParam(_uav_name_ + "/y", _pose_y_global_frame_);
      pl.loadParam(_uav_name_ + "/z", _pose_z_global_frame_);
      pl.loadParam(_uav_name_ + "/heading", _yaw_global_frame_);

      /* Check if all parameters were loaded successfully */ 
      if (!pl.loadedSuccessfully())
      {
        /* If not, alert the user and shut the node down */ 
        ROS_WARN("[CommonTfPub]: parameter loading failure");
        ros::shutdown();
      }

      // | ------------------ initialize subscribers ----------------- |

      // | ------------------ initialize publishers ----------------- |

      // | -------------------- initialize timers ------------------- |
      //
      timer_transformer_ = nh.createTimer(ros::Rate(_tf_timer_rate_), &CommonTfPub::callbackTimerCommonTfPub, this);
      // you can disable autostarting of the timer by the last argument

      // | --------------- initialize service servers --------------- |

      // | --------------- initialize service clients --------------- |

      // | ---------- initialize dynamic reconfigure server --------- |
      /* reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh)); */
      /* ReconfigureServer::CallbackType f = boost::bind(&CommonTfPub::callbackDynamicReconfigure, this, _1, _2); */
      /* reconfigure_server_->setCallback(f); */

      /* set the default value of dynamic reconfigure server to the value of parameter with the same name */
      /* { */
      /*   std::scoped_lock lock(mutex_waypoint_idle_time_); */
      /*   last_drs_config_.waypoint_idle_time = _waypoint_idle_time_; */
      /* } */
      /* reconfigure_server_->updateConfig(last_drs_config_); */

      ROS_INFO_ONCE("[CommonTfPub]: initialized");

      is_initialized_ = true;
    }
    //}

    // | ---------------------- msg callbacks --------------------- |
    // | --------------------- timer callbacks -------------------- |

    /* callbackTimerCommonTfPub() //{ */

    void CommonTfPub::callbackTimerCommonTfPub([[maybe_unused]] const ros::TimerEvent& te) {

      if (!is_initialized_)
        return;

      static tf2_ros::TransformBroadcaster broadcaster;
      geometry_msgs::TransformStamped transform_stamped;


      try{
        transform_stamped = tfBuffer.lookupTransform(_origin_frame_id_, _uav_frame_id_, ros::Time(0));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("[CommonTfPub]: Failed to get tranform. Execption: %s",ex.what());
      }

      if((transform_stamped.header.stamp.toSec() - ros::Time::now().toSec()) >= _invalidate_time_) {
        ROS_WARN("[CommonTfPub]: The global frame transformation if built on old data");
      }

      tf2::Quaternion q1{transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w};
      tf2::Vector3 v1{transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z};

      tf2::Quaternion q2; 
      q2.setRPY(0, 0, _yaw_global_frame_);
      tf2::Vector3 v2{_pose_x_global_frame_, _pose_y_global_frame_, _pose_z_global_frame_};

      tf2::Transform tf_uav_to_local, tf_global_to_local, tf_global_to_uav;
      tf_uav_to_local = tf2::Transform(q1, v1);
      tf_global_to_local = tf2::Transform(q2, v2);

      tf_global_to_uav.mult(tf_global_to_local, tf_uav_to_local);
      /* tf_global_to_uav = tf_uav_to_local; */

      transform_stamped.header.frame_id = _global_origin_frame_id_;
      transform_stamped.header.stamp = ros::Time::now();
      transform_stamped.child_frame_id = _uav_frame_id_;
      transform_stamped.transform.translation.x = tf_global_to_uav.getOrigin().x();
      transform_stamped.transform.translation.y = tf_global_to_uav.getOrigin().y();
      transform_stamped.transform.translation.z = tf_global_to_uav.getOrigin().z();
      transform_stamped.transform.rotation.x = tf_global_to_uav.getRotation().getX();
      transform_stamped.transform.rotation.y = tf_global_to_uav.getRotation().getY();
      transform_stamped.transform.rotation.z = tf_global_to_uav.getRotation().getZ();
      transform_stamped.transform.rotation.w = tf_global_to_uav.getRotation().getW();

      broadcaster.sendTransform(transform_stamped);
    }

    //}

    // | -------------------- service callbacks ------------------- |
    // | -------------- dynamic reconfigure callback -------------- |
    // | -------------------- support functions ------------------- |
    // | -------------------- compute functions ------------------- |

  }
}

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(swarm_utils::nodes::CommonTfPub, nodelet::Nodelet);
