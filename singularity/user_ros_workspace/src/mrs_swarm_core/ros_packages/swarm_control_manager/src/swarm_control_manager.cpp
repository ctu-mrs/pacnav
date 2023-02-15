#include <swarm_control_manager/swarm_control_manager.h>

/* limitVector() //{ */

static Eigen::Vector3d limitVector(const Eigen::Vector3d& vec, double min_value, double max_value) {
  double vec_norm = vec.norm();
  if (vec_norm > max_value) return vec / vec_norm * max_value;
  if (vec_norm < min_value) return Eigen::Vector3d::Zero();
  return vec;
}

//}

/* stringInVector() //{ */

bool stringInVector(const std::string &value, const std::vector<std::string> &vector) {

  if (std::find(vector.begin(), vector.end(), value) == vector.end()) {
    return false;
  } else {
    return true;
  }
}

//}

namespace swarm_control_manager
{

  // TODO dynamic reconfigure

  // | ------------------------ plugin class ------------------------ |

  /* onInit() //{ */

  void SwarmControlManager::onInit() {

    is_initialized_ = false;
    got_uvdar_cb_ = false;
    got_shared_gps_cb_ = false;

    manager_state_.is_active = false;
    manager_state_.tracker_ready = false;

    ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
    ros::Time::waitForValid();

    // | ------------------------ load ros params ------------------------ |

    mrs_lib::ParamLoader param_loader(nh_, "SwarmControlManager");
    param_loader.loadParam("uav_name", _uav_name_);
    param_loader.loadParam("uav_frame_id", _uav_frame_id_);
    param_loader.loadParam("origin_frame_id", _origin_frame_id_);

    _allowed_data_.insert(std::pair< string, vector<string> >("self_data", param_loader.loadParam2< vector<string> >("allowed_data/self_data")));
    _allowed_data_.insert(std::pair< string, vector<string> >("neighbor_data", param_loader.loadParam2< vector<string> >("allowed_data/neighbor_data")));
    param_loader.loadParam("neighbor_localization_types", _neighbor_localization_types_);
    param_loader.loadParam("control_types", _control_types_);

    _control_timer_rate_ = param_loader.loadParam2<double>("control_timer_rate");
    _invalidate_time_ = param_loader.loadParam2<double>("invalidate_time");
    _traj_gen_time_ = param_loader.loadParam2<double>("traj_gen_time");
    _vel_max_ = param_loader.loadParam2<double>("vel_max");

    // | ------------------------ init TF and common data------------------------ |

    transformer_.reset(new mrs_lib::Transformer("SwarmControlManager"));
    transformer_->setLookupTimeout(ros::Duration(_invalidate_time_));

    common_data_.reset(new SwarmCommonDataHandler());
    common_data_->getSelfState = std::bind(&SwarmControlManager::getSelfState, this);
    common_data_->getSelfHectorMap = std::bind(&SwarmControlManager::getSelfHectorMap, this);
    common_data_->getSelfLaserScan = std::bind(&SwarmControlManager::getSelfLaserScan, this);
    common_data_->getNeighborStates = std::bind(&SwarmControlManager::getNeighborStates, this);

    // | ------------------------ subscibers ------------------------ |

    sub_odom_ = nh_.subscribe("odom_in", 1, &SwarmControlManager::callbackOdometry, this, ros::TransportHints().tcpNoDelay());

    sub_uvdar_filtered_ = nh_.subscribe("uvdar_in", 1, &SwarmControlManager::callbackUvdarFilteredStates, this, ros::TransportHints().tcpNoDelay());
    sub_shared_gps_ = nh_.subscribe("shared_gps_in", 1, &SwarmControlManager::callbackSharedGps, this, ros::TransportHints().tcpNoDelay());

    sub_hector_map_ = nh_.subscribe("hector_map_in", 1, &SwarmControlManager::callbackHectorMap, this, ros::TransportHints().tcpNoDelay());
    sub_laser_scan_ = nh_.subscribe("laser_scan_in", 1, &SwarmControlManager::callbackLaserScan, this, ros::TransportHints().tcpNoDelay());

    // | ------------------------ publishers ------------------------ |

    pub_vel_ref_ = nh_.advertise<mrs_msgs::VelocityReferenceStamped>("vel_ref", 1);

    // | ------------------------ service servers ------------------------ |

    server_run_swarm_controller_ = nh_.advertiseService("run_controller", &SwarmControlManager::callbackRunSwarmController, this);
    server_activate_swarm_controller_ = nh_.advertiseService("activate_controller", &SwarmControlManager::callbackActivateSwarmController, this);

    // | ------------------------ service clients ------------------------ |

    client_switch_tracker_= nh_.serviceClient<mrs_msgs::String>("switch_tracker");
    client_path_ref_ = nh_.serviceClient<mrs_msgs::PathSrv>("path_ref");

    // | ------------------------ timers ------------------------ |

    timer_control_ = nh_.createTimer(ros::Rate(_control_timer_rate_), &SwarmControlManager::callbackControlTimer, this);

    // | ------------------------ plugin loader ------------------------ |

    controller_loader_ = std::make_unique<pluginlib::ClassLoader<SwarmController>>("swarm_control_manager", "swarm_control_manager::SwarmController");

    /* Load the plugin parameters//{ */

    vector<string> controller_names;
    param_loader.loadParam("controller_names", controller_names);

    for (const auto &it1 : controller_names) {
      std::string controller_name = it1;
      LoadedController lc;
      param_loader.loadParam(controller_name + "/address", lc.address);
      param_loader.loadParam(controller_name + "/ros_name_space", lc.ros_name_space);
      lc.required_data.insert(std::pair< string, vector<string> >("self_data", param_loader.loadParam2< vector<string> >(controller_name + "/required_data/self_data")));
      lc.required_data.insert(std::pair< string, vector<string> >("neighbor_data", param_loader.loadParam2< vector<string> >(controller_name + "/required_data/neighbor_data")));
      param_loader.loadParam(controller_name + "/neighbor_localization", lc.neighbor_localization);
      param_loader.loadParam(controller_name + "/control_type", lc.control_type);

      /* check loaded parameters//{ */
      if (!stringInVector(lc.neighbor_localization, _neighbor_localization_types_)) {
        ROS_ERROR_STREAM("[SwarmControlManager]: The neighbor localization '" << lc.neighbor_localization << "' is not a valid type, check neighbor_localization_types");
        ros::shutdown();
      }

      for (const auto &it2 : lc.required_data) {

        ROS_INFO_STREAM("[SwarmControlManager]: checking " + controller_name + "/required data/" << it2.first);
        vector<string> tmp_vec = it2.second;

        for (const auto &it3 : tmp_vec) {
          if (!stringInVector(it3, _allowed_data_[it2.first])) {
            ROS_ERROR_STREAM("[SwarmControlManager]: The " + controller_name + "/required data/" << it2.first << "/" << it3 << "is not a valid type, check allowed_data/" << it2.first);
            ros::shutdown();
          }
        }
      }

      if (!stringInVector(lc.control_type, _control_types_)) {
        ROS_ERROR_STREAM("[SwarmControlManager]: The control type '" << lc.control_type<< "' is not a valid type, check control_types");
        ros::shutdown();
      }
      //}

      loaded_controllers_[controller_name] = lc;

      if (!loadController(controller_name, loaded_controllers_[controller_name].control_type)) {
        ROS_ERROR_STREAM("[SwarmControlManager]: Could not load controller");
      }
    }

    //}

    /* Initialize the plugins //{ */

    for (const auto &param: loaded_controllers_) {
      std::string name = param.first;
      try {
        loaded_controllers_[name].plugin->initialize(nh_, name, param.second.ros_name_space, transformer_);
        ROS_INFO_STREAM("[SwarmControlManager]: Successfully initialized controller '" << name<<"'");
      } catch (std::runtime_error& e) {
        ROS_INFO_STREAM("[SwarmControlManager]: Exception caught during initialization of controller '" << name<<"'");
        ROS_INFO_STREAM("[SwarmControlManager]: Error: " << e.what());
      }
    }

    //}

    /* Activate initial plugin //{ */

    param_loader.loadParam("initial_controller", current_controller_);
    if (!loaded_controllers_.count(current_controller_)) {
      ROS_ERROR_STREAM("[SwarmControlManager]: Could not find initial controller '" << current_controller_ <<"'");
      ros::shutdown();
    }

    try {
      loaded_controllers_[current_controller_].plugin->activate();
      ROS_INFO_STREAM("[SwarmControlManager]: Successfully activated controller '" << current_controller_ <<"'");
    } catch (std::runtime_error &e) {
      ROS_INFO_STREAM("[SwarmControlManager]: Exception caught during activation of controller '" << current_controller_ <<"'");
      ROS_INFO_STREAM("[SwarmControlManager]: Error: " << e.what());
      ros::shutdown();
    }

    //}

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[SwarmControlManager]: Failed to load some parameters");
      ros::shutdown();
    }
    is_initialized_ = true;
  }

  //}

  // | ---------------------- topic callbacks --------------------- |

  /* callbackOdometry() //{ */

  void SwarmControlManager::callbackOdometry(const mrs_msgs::PositionCommandConstPtr& msg) 
  {
    if (!is_initialized_) return;

    std::scoped_lock lock(mutex_self_state_);

    /* transform to origin frame//{ */
    //transform pose
    mrs_msgs::ReferenceStamped pose_ref;
    pose_ref.header = msg->header;
    pose_ref.reference.heading = msg->heading;
    pose_ref.reference.position = msg->position;

    auto transformed_pose = transformer_->transformSingle(pose_ref, _origin_frame_id_);
    if (!transformed_pose) {
      ROS_ERROR_STREAM("[SwarmControlManager]: Cannot lookup transform for odometry pose from position command frame " << msg->header.frame_id << " to required frame " << _origin_frame_id_);
      self_state_.reset();
      return;
    }

    // transform veolcity
    geometry_msgs::Vector3Stamped velocity;
    velocity.header = msg->header;
    velocity.vector = swm_r_utils::createVector3(msg->velocity.x, msg->velocity.y, msg->velocity.z);

    auto transformed_velocity = transformer_->transformSingle(velocity, _origin_frame_id_);
    if (!transformed_velocity) {
      ROS_ERROR_STREAM("[SwarmControlManager]: Cannot transform odometry velocity from position command frame " << msg->header.frame_id << " to required frame " << _origin_frame_id_);
      self_state_.reset();
      return;
    }
    //}

    swm_utils::IdStateStamped self_state;
    self_state.header = transformed_pose->header;
    self_state.state.id = -1;
    self_state.state.pose.position = transformed_pose->reference.position;
    self_state.state.heading = transformed_pose->reference.heading;
    self_state.state.velocity.linear = transformed_velocity->vector;

    // Save the position and velocity
    self_state_ = boost::make_shared<swm_utils::IdStateStamped>(self_state);
  }

  //}

  /* callbackHectorMap() //{ */

  void SwarmControlManager::callbackHectorMap(const nav_msgs::OccupancyGridConstPtr& msg) {
    if (!is_initialized_) return;

    std::scoped_lock lock(mutex_self_hector_map_);

    /* transform to origin frame//{ */
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->info.origin;
    auto transformed_pose = transformer_->transformSingle(pose, _origin_frame_id_);
    if (!transformed_pose.has_value()) {
      ROS_ERROR_STREAM("[SwarmControlManager]: Cannot transform from occupancy grid frame " << msg->header.frame_id << " to  required frame " << _origin_frame_id_);
      self_hector_map_.reset();
      return;
    }
    //}

    nav_msgs::OccupancyGrid map_msg;
    map_msg.header = transformed_pose->header;
    map_msg.info = msg->info;
    map_msg.info.origin = transformed_pose->pose;
    map_msg.data = msg->data;

    // Save the map
    self_hector_map_ = boost::make_shared<nav_msgs::OccupancyGrid>(map_msg);
  };

  //}

  /* callbackLaserScan() //{ */

  void SwarmControlManager::callbackLaserScan(const sensor_msgs::LaserScanConstPtr& msg) {
    if (!is_initialized_) return;

    std::scoped_lock lock(mutex_self_laser_scan_);
    self_laser_scan_ = boost::make_shared<sensor_msgs::LaserScan>(*msg);
  }

  //}

  /* callbackUvdarFilteredStates() //{ */

  void SwarmControlManager::callbackUvdarFilteredStates(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& msg) {
    if (!is_initialized_) return;

    std::scoped_lock lock(mutex_neighbor_states_);

    swm_utils::IdStateArrayStamped neighbor_states;
    neighbor_states.header.frame_id = _origin_frame_id_;
    neighbor_states.header.stamp = msg->header.stamp;

    /* transform to origin frame//{ */
    //transform pose
    auto ret = transformer_->getTransform(msg->header.frame_id, _origin_frame_id_, msg->header.stamp);
    if(!ret) {
      ROS_ERROR_STREAM("[SwarmControlManager]: Cannot transform uvdar neighbor states from frame " << msg->header.frame_id << " to required frame " << _origin_frame_id_);
      got_uvdar_cb_ = false;
      return;
    }
    else {
      auto tf = ret.value();

      for(size_t i = 0; i < msg->poses.size(); i++) {
        mrs_msgs::ReferenceStamped pose_ref;
        pose_ref.header = msg->header;
        pose_ref.reference.position = msg->poses[i].pose.position;

        auto transformed_pose = transformer_->transform(pose_ref, tf);

        if (!transformed_pose) {
          ROS_ERROR_STREAM("[SwarmControlManager]: Cannot transform uvdar neighbor states from frame " << msg->header.frame_id << " to required frame " << _origin_frame_id_);
          got_uvdar_cb_ = false;
          return;
        }
        else {
          swm_utils::IdState tmp;
          tmp.id = msg->poses[i].id;
          tmp.pose.position = transformed_pose->reference.position;
          tmp.velocity.linear = swm_r_utils::createVector3(0, 0, 0);
          tmp.heading = 0.0;

          neighbor_states.states.push_back(tmp);
        }
      }
    }
    //}

    neighbor_states_ = boost::make_shared<swm_utils::IdStateArrayStamped>(neighbor_states);
    got_uvdar_cb_ = true;
  }

  //}

  /* callbackSharedGps() //{ */

  void SwarmControlManager::callbackSharedGps(const mrs_msgs::PoseWithCovarianceArrayStampedConstPtr& msg) {
    if (!is_initialized_) return;

    std::scoped_lock lock(mutex_neighbor_states_);

    swm_utils::IdStateArrayStamped neighbor_states;
    neighbor_states.header.frame_id = _origin_frame_id_;
    neighbor_states.header.stamp = msg->header.stamp;

    /* transform to origin frame//{ */
    auto ret = transformer_->getTransform(msg->header.frame_id, _origin_frame_id_);

    if(!ret) {

      ROS_ERROR_STREAM("[SwarmControlManager]: Cannot lookup transform between shared gps neighbor states from frame " << msg->header.frame_id << " to required frame " << _origin_frame_id_);
      got_shared_gps_cb_ = false;
      return;
    }
    else {
      auto tf = ret.value();

      for(size_t i = 0; i < msg->poses.size(); i++) {
        mrs_msgs::ReferenceStamped pose_ref;
        pose_ref.header = msg->header;
        pose_ref.reference.position = msg->poses[i].pose.position;

        auto transformed_pose = transformer_->transform(pose_ref, tf);

        if (!transformed_pose) {

          ROS_ERROR_STREAM("[SwarmControlManager]: Cannot transform shared gps neighbor states from frame " << msg->header.frame_id << " to required frame " << _origin_frame_id_);
          got_shared_gps_cb_ = false;
          return;
        }
        else {
          swm_utils::IdState tmp;
          tmp.id = msg->poses[i].id;
          tmp.pose.position = transformed_pose->reference.position;
          tmp.velocity.linear = swm_r_utils::createVector3(0, 0, 0);
          tmp.heading = 0.0;

          neighbor_states.states.push_back(tmp);
        }
      }
    }
    //}

    neighbor_states_ = boost::make_shared<swm_utils::IdStateArrayStamped>(neighbor_states);
    got_shared_gps_cb_ = true;
  }

  //}

  // | ---------------------- service server callbacks --------------------- |

  /* callbackRunSwarmController() //{ */

  bool SwarmControlManager::callbackRunSwarmController([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized_) return false;

    std::scoped_lock lock(mutex_manager_state_);
    if(manager_state_.is_active) {
      res.success = false;
      res.message = "can not activate swarm controller, it is already active";
      ROS_WARN_STREAM("[SwarmControlManager]: " << res.message);
    }
    else {
      manager_state_.is_active = true;

      res.success = true;
      res.message = "Activating swarm controller";
      ROS_INFO_STREAM("[SwarmControlManager]: " << res.message);
    }

    return true;
  }

  //}

  /* callbackActivateSwarmController() //{ */

  bool SwarmControlManager::callbackActivateSwarmController(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    if (!is_initialized_) return false;

    std::scoped_lock lock(mutex_manager_state_);
    if(req.data) {
      if(manager_state_.is_active) {
        res.success = false;
        res.message = "can not activate swarm controller, it is already active";
        ROS_WARN_STREAM("[SwarmControlManager]: " << res.message);
      }
      else {
        manager_state_.is_active = true;

        res.success = true;
        res.message = "Activating swarm controller";
        ROS_INFO_STREAM("[SwarmControlManager]: " << res.message);
      }
    }
    else { 

      if(!manager_state_.is_active) {
        res.success = false;
        res.message = "can not de-activate swarm controller, it is not active";
        ROS_WARN_STREAM("[SwarmControlManager]: " << res.message);
      }
      else {
        manager_state_.is_active = false;

        res.success = true;
        res.message = "De-activating swarm controller";
        ROS_INFO_STREAM("[SwarmControlManager]: " << res.message);
      }
    }

    return true;
  }

  //}

  // | ---------------------- routines --------------------- |

  /* loadController() //{ */

  bool SwarmControlManager::loadController(std::string &controller_name, std::string &control_type) {

    if (!loaded_controllers_.count(controller_name)) {
      ROS_ERROR_STREAM("[SwarmControlManager]: Parameters for controller" << controller_name << " were not defined");
      return false;
    }

    std::string address = loaded_controllers_[controller_name].address;
    try {

      if (control_type == "path" || control_type == "velocity") {

        loaded_controllers_[controller_name].plugin = controller_loader_->createInstance(address.c_str());
      }
      else {
        ROS_ERROR_STREAM("[SwarmControlManager]: Invalid control type '" << control_type << "' used for controller plugin '" << controller_name << "'");
        return false;
      }

      ROS_INFO_STREAM("[SwarmControlManager]: Loaded controller " << controller_name << " at address '" << address << "'");
      return true;
    }
    catch (pluginlib::CreateClassException& e) {
      ROS_ERROR_STREAM("[SwarmControlManager]: CreateClassException for controller " << controller_name << " at address '" << address << "'");
      ROS_ERROR_STREAM("[SwarmControlManager]: Error: " << e.what());
      return false;
    }
    catch (pluginlib::PluginlibException& e) {
      ROS_ERROR_STREAM("[SwarmControlManager]: PluginlibException for controller " << controller_name << " at address '" << address << "'");
      ROS_ERROR_STREAM("[SwarmControlManager]: Error: " << e.what());
      return false;
    }
  }

  //}

  /* isManagerReady() //{ */

  bool SwarmControlManager::isManagerReady(std::string &current_controller) {

    scoped_lock manager_lock(mutex_manager_state_);

    if (manager_state_.is_active) {

      if (!manager_state_.tracker_ready) {

        if(switchTracker("MpcTracker")) {
          manager_state_.tracker_ready = true;
          ROS_INFO_STREAM("[SwarmControlManager]: Activating controller '" + current_controller + "'");
          return true;
        }
        else {
          ROS_ERROR_STREAM("[SwarmControlManager]: Unable to activate controller '" + current_controller + "', tracker switching error!");
          return false;
        }
      }

      return true;
    }
    else {
      ROS_WARN_THROTTLE(2, "[SwarmControlManager]: Manager not ready, activate using activation serivce");
      return false;
    }
  }

  //}

  /* switchTracker() //{ */

  bool SwarmControlManager::switchTracker(const string tracker_name) {
    mrs_msgs::String switch_to_tracker;
    switch_to_tracker.request.value = tracker_name;

    client_switch_tracker_.call(switch_to_tracker);

    if(switch_to_tracker.response.success != true) {
      ROS_WARN("[SwarmControlManager]: Can not switch to %s", tracker_name.c_str());
      return false;
    }
    else {
      ROS_INFO("[SwarmControlManager]: Switched to %s", tracker_name.c_str());
      return true;
    }
  }

  //}

  // | ---------------------- getter routines --------------------- |

  /* getSelfState//{ */
  swm_utils::IdStateStampedConstPtr SwarmControlManager::getSelfState() {
    std::scoped_lock lock(mutex_self_state_);

    if (!is_initialized_) {
      self_state_.reset();
    }

    if(self_state_) {
      if((ros::Time::now().toSec() - self_state_->header.stamp.toSec()) > _invalidate_time_) {
        ROS_WARN_STREAM("[SwarmControlManager]: Self state data has stamp " << self_state_->header.stamp.toSec() << ", clearing data");
        self_state_.reset();
      }
    }
    else {
      ROS_WARN("[SwarmControlManager]: Can not provide self state to controller plugin, missing odom");
    }
    return self_state_;
  }
  //}

  /* getSelfHectorMap//{ */
  nav_msgs::OccupancyGridConstPtr SwarmControlManager::getSelfHectorMap() {

    std::scoped_lock lock(mutex_self_hector_map_);

    if (!is_initialized_) {
      self_hector_map_.reset();
    }

    if(stringInVector("hector_map", loaded_controllers_[current_controller_].required_data["self_data"])) {
      if(self_hector_map_) {
        if((ros::Time::now().toSec() - self_hector_map_->header.stamp.toSec()) > _invalidate_time_) {
          ROS_WARN("[SwarmControlManager]: Hector map data is old, clearing data");
          self_hector_map_.reset();
        }
      }
      else {
        ROS_WARN("[SwarmControlManager]: Can not provide self hector map to controller plugin, missing map data");
      }
    }
    else {
      ROS_ERROR("[SwarmControlManager]: Can not provide self hector map to controller plugin, its not a part of required data, check config");
      self_hector_map_.reset();
    }
    return self_hector_map_;
  }
  //}

  /* getSelfLaserScan//{ */
  sensor_msgs::LaserScanConstPtr SwarmControlManager::getSelfLaserScan() {

    std::scoped_lock lock(mutex_self_laser_scan_);

    if (!is_initialized_) {
      self_laser_scan_.reset();
    }

    if(stringInVector("laser_scan", loaded_controllers_[current_controller_].required_data["self_data"])) {
      if(self_laser_scan_) {
        if((ros::Time::now().toSec() - self_laser_scan_->header.stamp.toSec()) > _invalidate_time_) {
          ROS_WARN("[SwarmControlManager]: Laser scan data is old, clearing data");
          self_laser_scan_.reset();
        }
      }
      else {
        ROS_WARN("[SwarmControlManager]: Can not provide self laser scan to controller plugin, missing laser data");
      }
    }
    else {
      ROS_ERROR("[SwarmControlManager]: Can not provide self laser scan to controller plugin, its not a part of required data, check config");
      self_laser_scan_.reset();
    }
    return self_laser_scan_;
  }
  //}

  /* getNeighborStates//{ */
  swm_utils::IdStateArrayStampedConstPtr SwarmControlManager::getNeighborStates() {

    std::scoped_lock lock(mutex_neighbor_states_);

    if (!is_initialized_) {
      neighbor_states_.reset();
    }

    if(loaded_controllers_[current_controller_].neighbor_localization == "uvdar") {

      if(got_uvdar_cb_ && neighbor_states_) {

        if((ros::Time::now().toSec() - neighbor_states_->header.stamp.toSec()) > _invalidate_time_) {
          ROS_WARN_STREAM("[SwarmControlManager]: UVDAR data has stamp " << neighbor_states_->header.stamp.toSec() << ", clearing data");
          neighbor_states_.reset();
        }
      }
      else {
        ROS_WARN("[SwarmControlManager]: Can not provide UVDAR neighbors states to controller plugin, missing data");
        neighbor_states_.reset();
      }
    }
    else {
      if(loaded_controllers_[current_controller_].neighbor_localization == "shared_gps") {

        if(got_shared_gps_cb_ && neighbor_states_) {

          if((ros::Time::now().toSec() - neighbor_states_->header.stamp.toSec()) > _invalidate_time_) {
            ROS_WARN_STREAM("[SwarmControlManager]: Shared GPS data haas stamp " << neighbor_states_->header.stamp.toSec() << ", clearing data");
            neighbor_states_.reset();
          }
        }
        else {
          ROS_WARN("[SwarmControlManager]: Can not provide Shared GPS neighbors states to controller plugin, missing data");
          neighbor_states_.reset();
        }
      }
      else {
        ROS_ERROR("[SwarmControlManager]: Invaid param, can not provide neighbors states to controller plugin.");
        neighbor_states_.reset();
      }
    }

    return neighbor_states_;
  }
  //}

  // | ---------------------- timer callbacks --------------------- |

  /* callbackControlTimer() //{ */

  void SwarmControlManager::callbackControlTimer(const ros::TimerEvent&) {

    auto self_state = mrs_lib::get_mutexed(mutex_self_state_, self_state_);

    ROS_WARN_COND(!self_state, "[SwarmControlManager]: Waiting for Odometry");

    if (!is_initialized_ || !self_state) return;

    std::optional<std::any> control_cmd;

    /* Get the control command from update()//{ */
    try {

      std::scoped_lock controller_lock(mutex_current_controller_);
      auto ret_cmd = loaded_controllers_[current_controller_].plugin->update(common_data_);

      if (!ret_cmd) {

        ROS_ERROR_STREAM_THROTTLE(2.0, "[SwarmControlManager]: Current controller " << current_controller_ << " did not return a command");
        return;
      }
      else {

        ROS_INFO_STREAM_THROTTLE(3.0, "[SwarmControlManager]: Received cmd from controller " << current_controller_);
        control_cmd = ret_cmd.value();
      }
    }
    catch (std::runtime_error &e) {

      ROS_ERROR_STREAM("[SwarmControlManager]: Current controller " << current_controller_ << " crashed on update()");
      ROS_ERROR_STREAM("[SwarmControlManager]: Error: " << e.what());
      ros::shutdown();
    }

    //}

    // Check if the manager is ready with correct trackers
    if(isManagerReady(current_controller_)) {

      /* Path control mode //{ */

      if(loaded_controllers_[current_controller_].control_type == "path") {

        mrs_msgs::Path rec_path = std::any_cast<mrs_msgs::Path>(control_cmd.value());
        mrs_msgs::PathSrv msg_path;
        msg_path.request.path = rec_path;

        ROS_DEBUG_STREAM_THROTTLE(2, "[SwarmControlManager]: Received path. \n" << 
            "Time: " << rec_path.header.stamp.toSec() << "\n" <<
            "Frame: " << rec_path.header.frame_id << "\n" <<
            "Number of path points: " << rec_path.points.size() << "\n");

        if ((rec_path.header.stamp.toSec() - ros::Time::now().toSec()) < _traj_gen_time_) {

          ROS_WARN_STREAM_THROTTLE(2.0, "[SwarmControlManager]: Received path from '" << current_controller_ <<
              "' has stamp: " << rec_path.header.stamp.toSec() << " s, should be: "
              << ros::Time::now().toSec() + _traj_gen_time_ << " s (cur time + traj gen time). Path will not be used");
          return;
        }

        /* transform to origin frame//{ */

        if(rec_path.header.frame_id != _origin_frame_id_) {

          msg_path.request.path.points.clear();

          auto cur_tf = transformer_->getTransform(rec_path.header.frame_id, _origin_frame_id_, ros::Time::now());
          for(const auto &ref_pt : rec_path.points) {

            mrs_msgs::ReferenceStamped received_ref = swm_r_utils::createReferenceStamped(rec_path.header, ref_pt);

            auto transformed_ref = transformer_->transform(received_ref, cur_tf.value());

            if (!transformed_ref) {

              ROS_ERROR_STREAM("[SwarmControlManager]: Could not transform from contorl command frame " << rec_path.header.frame_id << " to required frame " << _origin_frame_id_);
              return;
            } 
            else {
              msg_path.request.path.points.emplace_back(transformed_ref.value().reference);
            }
          }
        }

        //}

        /* modify the path msg */ 
        msg_path.request.path.header.frame_id = _origin_frame_id_;
        msg_path.request.path.fly_now = true;
        msg_path.request.path.loop = false;

        bool success = client_path_ref_.call(msg_path);

        if (!success) {

          ROS_ERROR("[SwarmControlManager]: Service call for trajectory failed");
        }
        else {

          if (!msg_path.response.success) {

            ROS_ERROR("[SwarmControlManager]: Service call for trajectory failed: '%s'", msg_path.response.message.c_str());
          }
          else {
            ROS_INFO_THROTTLE(3, "[SwarmControlManager]: Using swarm controller in 'path' control mode");
          }
        }
      }

      //}

      else {

        /* Velocity Control mode //{ */

        if (loaded_controllers_[current_controller_].control_type == "velocity") {

          mrs_msgs::VelocityReferenceStamped vel_ref_msg = std::any_cast<mrs_msgs::VelocityReferenceStamped>(control_cmd.value());

          ROS_DEBUG_STREAM_THROTTLE(2, "[SwarmControlManager]: Received velocity reference. \n" << 
              "Time: " << vel_ref_msg.header.stamp.toSec() << "\n" <<
              "Frame: " << vel_ref_msg.header.frame_id << "\n" <<
              "Velocity: (" << vel_ref_msg.reference.velocity.x << ", " << vel_ref_msg.reference.velocity.y << ", " << vel_ref_msg.reference.velocity.z << ")\n" <<
              "Use heading: " << (vel_ref_msg.reference.use_heading ? "true" : "false") << "\n" <<
              "Heading: " << vel_ref_msg.reference.heading << "\n" <<
              "Use altitude: " << (vel_ref_msg.reference.use_altitude ? "true" : "false") << "\n" <<
              "Altitude: " << vel_ref_msg.reference.altitude << "\n" <<
              "Heading rate: " << vel_ref_msg.reference.heading_rate << "\n" <<
              "Use heading rate: " << (vel_ref_msg.reference.use_heading_rate ? "true" : "false"));

          if ((ros::Time::now().toSec() - vel_ref_msg.header.stamp.toSec()) > _invalidate_time_) {

            ROS_WARN_STREAM("[SwarmControlManager]: Controller '" << current_controller_ << "' return msg is older than " << _invalidate_time_ << "s, will not be used");
            return;
          }

          /* transform to origin frame//{ */

          if(vel_ref_msg.header.frame_id != _origin_frame_id_) {

            geometry_msgs::Vector3Stamped vec = swm_r_utils::createVector3Stamped(vel_ref_msg.header, vel_ref_msg.reference.velocity);
            mrs_msgs::ReferenceStamped ref_heading = swm_r_utils::createReferenceStamped(vel_ref_msg.header, swm_r_utils::createReference(geometry_msgs::Point(), vel_ref_msg.reference.heading));

            auto transformed_vec = transformer_->transformSingle(vec, _origin_frame_id_);
            auto transformed_heading = transformer_->transformSingle(ref_heading, _origin_frame_id_);

            if (!transformed_vec || !transformed_heading) {

              ROS_ERROR_STREAM("[SwarmControlManager]: Could not transform from contorl command frame " << vel_ref_msg.header.frame_id << " to required frame " << _origin_frame_id_);
              return;
            } 
            else {

              vel_ref_msg.header = transformed_vec->header;
              vel_ref_msg.reference.velocity = transformed_vec->vector;
              vel_ref_msg.reference.heading = transformed_heading->reference.heading;
            }
          }

          //}

          // Limit the velocity
          Eigen::Vector3d lim_vel = limitVector(swm_r_utils::vector3ToEigen(vel_ref_msg.reference.velocity), 0.01, _vel_max_);

          // Create the mrs_msg

          if (lim_vel.allFinite() && isfinite(vel_ref_msg.reference.heading)) {

            vel_ref_msg.reference.velocity = swm_r_utils::vector3FromEigen(lim_vel);
          }
          else {

            ROS_ERROR_STREAM("[SwarmControlManager]: Velocity or heading has INF!");
            return;
          }

          // Publish it
          try {

            pub_vel_ref_.publish(vel_ref_msg);
            ROS_INFO_THROTTLE(3, "[SwarmControlManager]: Using swarm controller in velocity control mode");
          } catch (...) {

            ROS_ERROR_STREAM("[SwarmControlManager]: Exception caught during publishing topic " << pub_vel_ref_.getTopic());
          }
        }

        //}

      }
    }
  }

  //}

}

PLUGINLIB_EXPORT_CLASS(swarm_control_manager::SwarmControlManager, nodelet::Nodelet);
