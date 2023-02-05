/* include header file of this class */
#include "boids_controller.h"

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace boids
{

  /* initialize() //{ */

  void BoidsController::initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& ros_name_space, std::shared_ptr<mrs_lib::Transformer> transformer) {

    // | ---------------- set my booleans to false ---------------- |
    // but remember, always set them to their default value in the header file
    // because, when you add new one later, you might forget to come back here
    _name_ = name;

    // | ------------------------ Tf ------------------------------ |

    transformer_ = transformer;

    /* obtain node handle */
    ros::NodeHandle nh(parent_nh, ros_name_space);
    ros::NodeHandle nh_g("/");

    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();

    // | ------------------- load ros parameters ------------------ |
    /* (mrs_lib implementation checks whether the parameter was loaded or not) */
    mrs_lib::ParamLoader pl_parent(parent_nh, "BoidsController");

    pl_parent.loadParam("uav_name", _uav_name_);
    pl_parent.loadParam("uav_frame_id", _uav_frame_id_);
    pl_parent.loadParam("origin_frame_id", _origin_frame_id_);

    mrs_lib::ParamLoader pl_child(nh, "BoidsController");
    _uav_radius_ = pl_child.loadParam2<double>("uav_radius");
    _lidar_range_ = pl_child.loadParam2<double>("lidar_range");
    _invalidate_time_ = pl_child.loadParam2<double>("invalidate_time");
    _takeoff_height_ = pl_child.loadParam2<double>("takeoff_height");

    _prox_vec_zero_dist_ = pl_child.loadParam2<double>("prox_vec_zero_dist");
    _prox_vec_crit_dist_ = pl_child.loadParam2<double>("prox_vec_crit_dist");
    _prox_vec_rep_const_ = pl_child.loadParam2<double>("prox_vec_rep_const");

    _prox_vec_follow_dist_ = pl_child.loadParam2<double>("prox_vec_follow_dist");
    _prox_vec_coh_const_ = pl_child.loadParam2<double>("prox_vec_coh_const");

    _col_vec_const_ = pl_child.loadParam2<double>("col_vec_const");
    _col_vec_max_dist_ = pl_child.loadParam2<double>("col_vec_max_dist");

    _k_prox_ = pl_child.loadParam2<double>("k_prox");
    _k_col_ = pl_child.loadParam2<double>("k_col");
    _k_nav_ = pl_child.loadParam2<double>("k_nav");

    _dir_vel_mag_ = pl_child.loadParam2<double>("dir_vel_mag");
    _dir_invalid_time_ = pl_child.loadParam2<int>("dir_invalid_time");

    // | ------------------ initialize subscribers ----------------- |

    sub_des_path_ = nh_g.subscribe(pl_child.loadParam2<string>("path_in"), 1, &BoidsController::callbackPath, this, ros::TransportHints().tcpNoDelay());
    sub_des_dir_ = nh_g.subscribe(pl_child.loadParam2<string>("dir_in"), 1, &BoidsController::callbackDesDirection, this, ros::TransportHints().tcpNoDelay());

    mrs_lib::SubscribeHandlerOptions shopts(nh_g);
    shopts.node_name = name;
    shopts.threadsafe = true;
    shopts.no_message_timeout = ros::Duration(5.0);
    shopts.use_thread_timer = false;

    mrs_lib::construct_object(sh_human_pose_,
        shopts,
        pl_child.loadParam2<string>("human_in")
        );

    // | ------------------ initialize publishers ----------------- |

    pub_viz_proximal_vec_ = nh.advertise<visualization_msgs::Marker>("viz/proximal_ctrl_vec", 1);
    pub_viz_collision_vec_ = nh.advertise<visualization_msgs::Marker>("viz/collision_ctrl_vec", 1);
    pub_viz_nav_vec_ = nh.advertise<visualization_msgs::Marker>("viz/nav_ctrl_vec", 1);
    pub_viz_dir_vec_ = nh.advertise<visualization_msgs::Marker>("viz/dir_ctrl_vec", 1);
    pub_viz_final_vec_ = nh.advertise<visualization_msgs::Marker>("viz/final_vec", 1);
    pub_viz_col_vec_list_ = nh.advertise<visualization_msgs::MarkerArray>("viz/col_vec_list", 1);
    pub_viz_vir_obst_ = nh.advertise<visualization_msgs::Marker>("viz/vir_obst", 1);
    pub_mrs_uav_status_ = nh_g.advertise<std_msgs::String>(pl_child.loadParam2<string>("mrs_uav_status_out"), 1);

    /* Check if all parameters were loaded successfully */ 
    if (!pl_parent.loadedSuccessfully() || !pl_child.loadedSuccessfully())
    {
      /* If not, alert the user and shut the node down */ 
      ROS_ERROR("[BoidsController] parameter loading failure");
      ros::shutdown();
    }

    // | ---------- initialize dynamic reconfigure server --------- |

    /* reconfig_srv_.reset(new dynamic_reconfigure::Server<multi_uav_dynreconfig::DynReconfigConfig>(mutex_dyn_reconfig_, nh)); */
    /* dynamic_reconfigure::Server<multi_uav_dynreconfig::DynReconfigConfig>::CallbackType f = boost::bind(&BoidsController::callbackDynReconfig, this, _1, _2); */
    /* reconfig_srv_->setCallback(f); */

    final_vec_.header.stamp = ros::Time::now();
    final_vec_.header.frame_id = _uav_frame_id_;

    des_dir_ = 0;
    dir_map_["forward"] = 1;
    dir_map_["left"] = 2;
    dir_map_["backward"] = 3;
    dir_map_["right"] = 4;

    is_init_ = true;
    ROS_INFO_ONCE("[BoidsController] Initialized");
  }
  //}

  /* activate() //{ */

  void BoidsController::activate() {
    is_active_ = true;
    ROS_INFO_STREAM("[BoidsController] Activated");
  }

  //}

  /* deactivate() //{ */

  void BoidsController::deactivate() {
    is_active_ = false;
    ROS_INFO_STREAM("[BoidsController] Deactivated");
  }

  //}
  
  /* update() //{ */
  std::optional<std::any> BoidsController::update(std::shared_ptr<swm_ctrl::SwarmCommonDataHandler> common_data) {

    if (!is_init_ || !is_active_) { 
      return nullopt;
    }

    e::Vector3d rel_human_vec{0, 0, 0};
    if (sh_human_pose_.newMsg()) {
      geometry_msgs::PoseWithCovarianceStampedConstPtr rec_human_pose = sh_human_pose_.getMsg();

      mrs_msgs::ReferenceStamped tmp = swm_r_utils::createReferenceStamped(rec_human_pose->header, swm_r_utils::createReference(rec_human_pose->pose.pose.position, 0.0));
      auto res = transformer_->transformSingle(tmp, _uav_frame_id_);

      if(res) {
        rel_human_vec = swm_r_utils::pointToEigen(res.value().reference.position);
      }
      else {
        ROS_ERROR("[PacnavController] Tf 'human pose' failed. From %s to %s", rec_human_pose->header.frame_id.c_str(), _uav_frame_id_.c_str());
        return nullopt;
      }
    }

    auto ret = calcFinalControlVec(final_vec_, des_dir_, common_data); 
    if(ret) {
      final_vec_ = ret.value();
    } else {
      return nullopt;
    }

    /* reset the direction after invalid time */
    if ((ros::Time::now().toSec() - dir_cmd_time_.toSec()) > _dir_invalid_time_) {
      des_dir_ = 0;
    }

    swm_utils::IdStateArrayStampedConstPtr neighbor_states = common_data->getNeighborStates();
    if(!neighbor_states) {
      ROS_ERROR("[BoidsController] Did not receive any neighbor states");
      return nullopt;
    }

    /* prepare the cotnrol cmd for swarm_control_manager */
    mrs_msgs::VelocityReferenceStamped vel_cmd;
    vel_cmd.header = final_vec_.header;
    vel_cmd.reference.velocity = final_vec_.vector;
    vel_cmd.reference.heading = 0.0;

    vel_cmd.reference.use_heading_rate = false;
    vel_cmd.reference.use_heading = false;

    return std::any{vel_cmd};
  }

  //}

  // | ---------------------- callbacks --------------------- |

  /* callbackPath() //{ */
  void BoidsController::callbackPath(nav_msgs::PathConstPtr msg) {
    /* do not continue if the nodelet is not initialized */
    if (!is_init_)
      return;

    des_path_ = boost::make_shared<nav_msgs::Path>(*msg);
  }

  //}
  
  /* callbackDesDirection() //{ */
  void BoidsController::callbackDesDirection(std_msgs::StringConstPtr msg) {
    if(!is_init_)
      return;

    if (des_dir_ == 0) {

      if (dir_map_.find(msg->data) != dir_map_.end()) {
        des_dir_ = dir_map_[msg->data];
        dir_cmd_time_ = ros::Time::now();
        ROS_WARN_STREAM("[BoidsController] Recieved direction cmd: " << msg->data);
      }
      else {
        ROS_ERROR_STREAM("[BoidsController] Recieved Invalid direction cmd: " << msg->data);
      }
    }
  }
  //}

  // | -------------- dynamic reconfigure callback -------------- |

  /* /1* callbackDynReconfig() //{ *1/ */

  /* void BoidsController::callbackDynReconfig([[maybe_unused]] multi_uav_dynreconfig::DynReconfigConfig& config, [[maybe_unused]] uint32_t level) { */
  /*   if (!is_init_) */
  /*     return; */

  /*   { */
  /*     scoped_lock lock(mutex_dyn_params_); */
  /*   _prox_vec_rep_const_        = config.uav_f_rep_const; */
  /*   _prox_vec_crit_dist_ = config.uav_f_crit_dist; */
  /*   _prox_vec_zero_dist_                = config.uav_f_zero_dist; */

  /*   _prox_vec_follow_dist_ = config.uav_f_fol_dist; */
  /*   _prox_vec_coh_const_        = config.uav_f_coh_const; */

  /*   _col_vec_const_            = config.col_vec_const; */
  /*   _col_vec_max_dist_        = config.col_vec_max_dist; */

  /*   /1* _nav_force_position_constant_        = config.nav_force_position_constant; *1/ */
  /*   /1* _nav_force_velocity_constant_        = config.nav_force_velocity_constant; *1/ */
  /*   _k_prox_ = config.k_prox; */
  /*   _k_nav_ = config.k_nav; */
  /*   _k_col_ = config.k_col; */
  /*   _k_drag_ = config.k_drag; */

  /*   _force_max_value_                     = config.force_max_value; */
  /*   _vel_max_value_                  = config.vel_max_value; */
  /*   _force_min_value_ = config.force_min_value; */
  /*   _vel_min_value_ = config.vel_min_value; */
  /*   } */
  /* } */

  /* //} */

  // | -------------------- utility functions ------------------- |

  /* | -------------------- vector utils ------------------- | //{*/

/* pubVecViz() //{ */ 
  void BoidsController::pubVecViz(geometry_msgs::Vector3Stamped ctrl_vec, ros::Publisher& pub, std_msgs::ColorRGBA color) {

  visualization_msgs::Marker viz;

  viz.header = ctrl_vec.header;
  viz.ns = "Control Vector";
  viz.id = 0;
  viz.type = visualization_msgs::Marker::ARROW;
  viz.action = visualization_msgs::Marker::ADD;
  viz.scale.x = 0.04;
  viz.scale.y = 0.07;
  viz.color = color;
  viz.pose.orientation.w = 1.0;

  geometry_msgs::Point vec_tail, vec_head;
  vec_tail.x = 0.0;
  vec_tail.y = 0.0;
  vec_tail.z = -_takeoff_height_;
  vec_head.x = ctrl_vec.vector.x;
  vec_head.y = ctrl_vec.vector.y;
  vec_head.z = -_takeoff_height_;
  
  viz.points.push_back(vec_tail);
  viz.points.push_back(vec_head);

  pub.publish(viz);
  }

  //}

//}

/* | -------------------- collision control utils ------------------- | //{*/

  /* angle2Index() //{ */
  int BoidsController::angle2Index(double angle, int max_index) {

    int index;

    if(angle >= 0.0) {
      index = int(((max_index/2.0)/M_PI) * angle);
    }
    else {
      index = max_index - int(((max_index/2.0)/-M_PI) * angle);
    }

    return index;
  }

  //}

  /* index2Angle() //{ */
  double BoidsController::index2Angle(int index, int max_index) {

    double angle = 0;

    if(index <= max_index) {
      angle = double((2 * M_PI * index)/max_index);
    }
    else {
      ROS_ERROR("[BoidsController] Index out of range for angle conversion");
    }

    return angle;
  }

  //}

  /* cleanLidarData()  //{ */
  vector<double> BoidsController::cleanLidarData(sensor_msgs::LaserScanConstPtr lidar_data, double max_range){
    vector<double> lidar_ranges;
    vector<float> temp_data = lidar_data->ranges;
    size_t lsz = lidar_data->ranges.size();
    lidar_ranges.resize(lsz);

    for(size_t i = 0; i < lsz; i++) {
      lidar_ranges[(int(lsz/2.0) + i) % lsz] = isfinite(temp_data[i]) ? double(temp_data[i]) : 2*max_range;
    }

    //Filtering input lidar data 
    vector<double> filt_lidar;
    int filt_sz = 5;
    filt_lidar.resize(lsz);
    e::VectorXd filt_mask;
    filt_mask.resize(filt_sz, 1);
    filt_mask << 1,1,1,1,1;
    for(size_t i = 0; i < lsz; i++) {
      e::VectorXd data;
      data.resize(filt_sz, 1);
      data[int(filt_sz/2)] = lidar_ranges[i];
      for(int j = 1; j <= int(filt_sz/2); j++) {
        data[int(filt_sz/2) - int(j)] = lidar_ranges[(i-j)%lsz];
        data[int(filt_sz/2) + int(j)] = lidar_ranges[(i+j)%lsz];
      }
      filt_lidar[i] = filt_mask.dot(data)/filt_sz;
    }
    lidar_ranges = filt_lidar;

    /* for(size_t i = 0; i < lsz; i++) { */
    /*   msg_mod_lidar_.ranges[(int(lsz/2.0) + i) % lsz] = lidar_ranges[i]; */
    /* } */
    /* pub_mod_lidar_.publish(msg_mod_lidar_); */

    return lidar_ranges;
  } 
  //}

  /* getObstFromLidar()  //{ */
  vector<e::Vector2d> BoidsController::getObstFromLidar(vector<double> lidar_ranges, double max_range){

    vector<e::Vector2d> vir_obst;

    for(size_t i = 0; i < lidar_ranges.size();) {
      if((lidar_ranges[i] - max_range) > 0.0) {
        ++i;
      }
      else {
        /* cout<<"Vir obst at index: "<<int(i)<<endl; */
        double theta = index2Angle(i, lidar_ranges.size());
        vir_obst.push_back(e::Vector2d{lidar_ranges[i] * cos(theta), lidar_ranges[i] * sin(theta)});
        i += 30;
      }
    }

    /* cout<<"Total no. of Virtual obst: "<<double(vir_obst.size())<<endl; */

    return vir_obst;
  } 
  //}

  /* getSteerVec()  //{ */
  e::Vector2d BoidsController::getSteerVec(e::Vector2d comp_vec, e::Vector2d base_vec, double des_mag, double des_dir){
    /* cout<<"Relative obst pose: ("<<base_vec(0)<<", "<<base_vec(1)<<") Cur total force: ("<<comp_vec(0)<<", "<<comp_vec(1)<<") Desired mag: "<<des_mag<<" and dir: "<<des_dir<<endl; */

    e::Rotation2Dd left(des_dir);
    e::Rotation2Dd right(-des_dir);
    e::Vector2d temp_left, temp_right;
    temp_left = left.toRotationMatrix() * -base_vec;
    temp_right = right.toRotationMatrix() * -base_vec;
    /* ROS_INFO("temp osbtacle pose left and right: %f, %f, %f, %f", temp_left(0),temp_left(1),temp_right(0),temp_right(1)); */

    if(comp_vec.norm() >= sqrt(3 * pow(0.01, 2))) {

      double l_dot_prdt = temp_left.normalized().dot(comp_vec.normalized());
      double r_dot_prdt = temp_right.normalized().dot(comp_vec.normalized());
      /* ROS_INFO("The left dot prod = %f, right dot prod = %f", left_dot_prdt, right_dot_prdt); */

      if((l_dot_prdt - r_dot_prdt) >= 0.0) {
        /* cout<<"Returning left vec"<<endl; */
        return e::Vector2d{des_mag * temp_left.normalized()};
      }
      else{
        /* cout<<"Returning right vec"<<endl; */
        return e::Vector2d{des_mag * temp_right.normalized()};
      }
    }
    else {
      /* cout<<"Returning -base vec"<<endl; */
      return e::Vector2d{-des_mag * base_vec.normalized()};
    }
  }
  //}

  //}

  /* | -------------------- nav control utils ------------------- | //{*/

  /* getNextNavPoint() //{ */
  std::optional<geometry_msgs::PointStamped> BoidsController::getNextNavPoint(swm_utils::IdStateStampedConstPtr self_state, nav_msgs::PathConstPtr des_path) {

    for(auto it = des_path->poses.begin(); it != des_path->poses.end(); it++) {
      e::Vector2d cand_pt{it->pose.position.x, it->pose.position.y};

      /* tranform to self_state frame//{ */
      mrs_msgs::ReferenceStamped tmp = swm_r_utils::createReferenceStamped(des_path->header, swm_r_utils::createReference(it->pose.position, 0.0));
      auto tf_pose = transformer_->transformSingle(tmp, self_state->header.frame_id);
      if(tf_pose) {
        cand_pt = e::Vector2d{tf_pose.value().reference.position.x, tf_pose.value().reference.position.y};
      }
      else {
        ROS_ERROR("[BoidsController] Tf 'des_path' failed. From %s to %s", des_path->header.frame_id.c_str(), self_state->header.frame_id.c_str());
        return nullopt;
      }//}

    if(((cand_pt - e::Vector2d{self_state->state.pose.position.x, self_state->state.pose.position.y}).norm() - (_uav_radius_/3.0)) > 0.0) {
      auto nav_pose = swm_r_utils::createPointStamped(it->header, it->pose.position);
      return nav_pose;
    }
    }
    return nullopt;
  }

  //}

  /* rescaleNavCtrlVec() //{ */
  e::Vector2d BoidsController::rescaleNavCtrlVec(const e::Vector2d nav_vec, swm_utils::IdStateArrayStampedConstPtr neighbor_states) {

    double avg_dist = 0;
    e::Vector2d result = nav_vec;

    /* get average dist of neighbor robots//{ */
    for(auto itr = neighbor_states->states.begin(); itr != neighbor_states->states.end(); itr++) {
      e::Vector2d pose = e::Vector2d{itr->pose.position.x, itr->pose.position.y};

      /* transform to uav frame//{ */
      mrs_msgs::ReferenceStamped tmp = swm_r_utils::createReferenceStamped(neighbor_states->header, swm_r_utils::createReference(itr->pose.position, 0.0));
      auto tf_pose = transformer_->transformSingle(tmp, _uav_frame_id_);
      if(tf_pose) {
        pose = e::Vector2d{tf_pose.value().reference.position.x, tf_pose.value().reference.position.y};
      }
      else {
        ROS_ERROR("[BoidsController] Tf 'neighbor_states' failed for 'rescaleNavCtrlVec'. From %s to %s", neighbor_states->header.frame_id.c_str(), _uav_frame_id_.c_str());
        return result;
      }//}

    double dist = pose.norm() - _uav_radius_;
    dist = (dist > 0.0) ? dist : 0.01;

    avg_dist += dist;
    }

    if(neighbor_states->states.size() > 0) {
      avg_dist /= neighbor_states->states.size();
    }//}

  double sf = -0.017*pow(avg_dist, 2.0) + 1.0;
  sf = (sf < 0.0) ? 0.0 : sf;
  ROS_INFO("[BoidsController] Rescaling Nav Control Vec by factor: %f", sf);
  result = sf * nav_vec;
  return result;
  }
  //}

  //}

  // | -------------------- functions ------------------- |

  /* calcProximalCtrlVec() //{ */
  geometry_msgs::Vector3Stamped BoidsController::calcProximalCtrlVec(swm_utils::IdStateArrayStampedConstPtr neighbor_states, string type) {

    geometry_msgs::Vector3Stamped result;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = _uav_frame_id_;
    result.vector.x = 0.0;
    result.vector.y = 0.0;
    result.vector.z = 0.0;

    e::Vector2d net_vec{0, 0};

    /* Quad-Expo force //{ */
    if(type == "quadexpo") {

      double k_1 = _prox_vec_rep_const_ * pow(_prox_vec_crit_dist_ - _prox_vec_zero_dist_,2) * (sqrt(_prox_vec_crit_dist_ * _prox_vec_zero_dist_)/(sqrt(_prox_vec_zero_dist_) - sqrt(_prox_vec_crit_dist_)));
      double k_4 = _prox_vec_coh_const_ * pow(_prox_vec_follow_dist_ - _prox_vec_zero_dist_, 2)/atan(0.1*_prox_vec_follow_dist_);

      uint8_t coh_uavs = 0;
      uint8_t rep_uavs = 0;
      e::Vector2d rep_vec{0, 0};
      e::Vector2d coh_vec{0, 0};

      for(auto itr = neighbor_states->states.begin(); itr != neighbor_states->states.end(); itr++) {
        e::Vector2d rel_pose = e::Vector2d{itr->pose.position.x, itr->pose.position.y};

        mrs_msgs::ReferenceStamped tmp = swm_r_utils::createReferenceStamped(neighbor_states->header, swm_r_utils::createReference(itr->pose.position, 0.0));
        auto res = transformer_->transformSingle(tmp, result.header.frame_id);
        if(res) {
          rel_pose = e::Vector2d{res.value().reference.position.x, res.value().reference.position.y};
        }
        else {
          ROS_ERROR("[BoidsController] Tf 'rel_pose' failed. From %s to %s", neighbor_states->header.frame_id.c_str(), _uav_frame_id_.c_str());
          return result;
        }

        double dist = rel_pose.norm() - _uav_radius_;
        dist = (dist > 0.0) ? dist : 0.01;

        ROS_INFO("[BoidsController] Distance of Uav %zi = %f", itr->id, dist);
        ROS_INFO("[BoidsController] Rel pose vec of Uav %zi X: %f, Y: %f", itr->id, rel_pose(0), rel_pose(1));
        ROS_INFO("[BoidsController] Using Quad-Expo function");

        // Repuslion vector 
        rep_vec -= ((dist < _prox_vec_crit_dist_) ? k_1 * (1.0/sqrt(dist) - 1.0/sqrt(_prox_vec_zero_dist_)) : ((dist < _prox_vec_zero_dist_) ? _prox_vec_rep_const_ * pow(dist - _prox_vec_zero_dist_, 2) : 0)) * rel_pose.normalized();
        /* ROS_INFO("Separation force: %f, %f, %f", separationForce(0), separationForce(1), separationForce(2)); */

        // Cohesion force
        coh_vec += ((dist < _prox_vec_zero_dist_) ? 0 : ((dist < _prox_vec_follow_dist_) ? _prox_vec_coh_const_ * pow(dist - _prox_vec_zero_dist_, 2) : k_4 * atan(dist - 0.9*_prox_vec_follow_dist_))) * rel_pose.normalized();
        /* ROS_INFO("Cohesionforce: %f, %f, %f", cohesionForce(0), cohesionForce(1), cohesionForce(2)); */

        if((dist - _prox_vec_zero_dist_) < 0.0) 
          ++rep_uavs;
        else
          ++coh_uavs;
      }
      /* ROS_INFO(" no of repel uavs: %u", repeling_uavs); */
      /* ROS_INFO(" no of cohesing uavs: %u", cohesing_uavs); */

      e::Vector2d mean_rep_vec = rep_vec;
      e::Vector2d mean_coh_vec = coh_vec;

      if(rep_uavs > 0)
        mean_rep_vec /= rep_uavs;

      if(coh_uavs > 0)
        mean_coh_vec /= coh_uavs;

      net_vec = mean_rep_vec + mean_coh_vec;
    }
    //}

    /* /1* Leanord-Jonnes vector//{ *1/ */
    /* if(type == "LJ") { */

    /*   for (size_t i = 0; i < team_states.size(); ++i) { */
    /*     ROS_INFO("[BoidsController] UAV %i coordinates X = %f, Y = %f, Z = %f", team_states[i].id, team_states[i].pose.position.x, team_states[i].pose.position.y, team_states[i].pose.position.z); */

    /*     e::Vector2d rel_pose = e::Vector2d{team_states[i].pose.position.x, team_states[i].pose.position.y}; */
    /*     if(team_states[i].header.frame_id != result.header.frame_id) { */
    /*       auto tf_pose = tfPose(team_states[i].header.frame_id, result.header.frame_id, rel_pose); */

    /*       if(tf_pose) { */
    /*         rel_pose = tf_pose.value(); */
    /*       } */
    /*       else { */
    /*         return result; */
    /*       } */
    /*     } */

    /*     double dist = rel_pose.norm() - _uav_radius_; */
    /*     dist = (dist > 0.0) ? dist : 0.01; */
    /*     ROS_INFO("[BoidsController] Distance of Uav %i = %f", team_states[i].id, dist); */
    /*     ROS_INFO("[BoidsController] Using Leonard-Jonnes force"); */

    /*     net_vec += -(4 * uav_f_coh_const * 2/dist) * (2 * pow((uav_f_zero_dist/dist), 2 * uav_f_coh_const) - pow((uav_f_zero_dist/dist), uav_f_coh_const)) * rel_pose.normalized(); */  
    /*   } */
    /* } */
    /* //} */

    if(!isfinite(net_vec(0)) || !isfinite(net_vec(1))) {
      net_vec = e::Vector2d{0, 0};
      ROS_WARN("[BoidsController] Proximal control vector is not finite");
    }

    result.vector.x = net_vec(0);
    result.vector.y = net_vec(1);
    result.vector.z = 0.0;

    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.1;
    color.g = 0.1;
    color.b = 0.8;

    pubVecViz(result, pub_viz_proximal_vec_, color);

    /* cout<<"UAV Force X: "<<result.vector.x<<" Y: "<<result.vector.y<<endl; */

    return result;
  }

  //}

  /* calcCollisionCtrlVec() //{ */
  geometry_msgs::Vector3Stamped BoidsController::calcCollisionCtrlVec(const geometry_msgs::Vector3Stamped pref_direction, sensor_msgs::LaserScanConstPtr lidar_data) {

    geometry_msgs::Vector3Stamped result;
    result.header.stamp = ros::Time::now();
    result.header.frame_id = _uav_frame_id_;
    result.vector.x = 0;
    result.vector.y = 0;
    result.vector.z = 0;

    if(!lidar_data) {
      ROS_WARN("[BoidsController] No lidar!, can not do collision avoidance");
      return result;
    }

    e::Vector2d net_vec{0, 0}, pref_vec;

    auto tf_vec = transformer_->transformSingle(pref_direction, result.header.frame_id);
    if(tf_vec) {
      pref_vec = e::Vector2d{tf_vec.value().vector.x, tf_vec.value().vector.y}; 
    }
    else {
      ROS_ERROR("[BoidsController] Tf 'pref_direction' failed. From %s to %s", pref_direction.header.frame_id.c_str(), result.header.frame_id.c_str());
      return result;
    }

    /* Viz init for info about obst and forces //{*/
    visualization_msgs::MarkerArray msg_viz;
    visualization_msgs::Marker viz_col_ctrl_vec;
    viz_col_ctrl_vec.header.stamp = ros::Time::now();
    viz_col_ctrl_vec.header.frame_id = result.header.frame_id;
    viz_col_ctrl_vec.ns = "Virtual Collision Avoidance Vectors";
    viz_col_ctrl_vec.id = 0;
    viz_col_ctrl_vec.type = visualization_msgs::Marker::ARROW;
    viz_col_ctrl_vec.action = visualization_msgs::Marker::ADD;
    viz_col_ctrl_vec.scale.x = 0.03;
    viz_col_ctrl_vec.scale.y = 0.06;
    viz_col_ctrl_vec.pose.orientation.w = 1.00;
    viz_col_ctrl_vec.color.a = 1.0; // Don't forget to set the alpha!
    viz_col_ctrl_vec.color.r = 0.0;
    viz_col_ctrl_vec.color.g = 0.2;
    viz_col_ctrl_vec.color.b = 0.4;

    visualization_msgs::Marker viz_vir_obst;
    viz_vir_obst.header.stamp = ros::Time::now();
    viz_vir_obst.header.frame_id = result.header.frame_id;
    viz_vir_obst.ns = "Virtual Obstacles";
    viz_vir_obst.id = 0;
    viz_vir_obst.type = visualization_msgs::Marker::LINE_LIST;
    viz_vir_obst.action = visualization_msgs::Marker::ADD;
    viz_vir_obst.scale.x = 0.03;
    viz_vir_obst.pose.orientation.w = 1.00;
    viz_vir_obst.color.a = 1.0; // Don't forget to set the alpha!
    viz_vir_obst.color.g = 0.4;
    viz_vir_obst.color.r = 0.7;

    //}

    vector<e::Vector2d> static_obst = getObstFromLidar(cleanLidarData(lidar_data, _lidar_range_), _col_vec_max_dist_ + _uav_radius_);

    /* Collision avoidance vector calc for static obst//{*/
    ROS_INFO("[BoidsController] Calculating Collision control vector for static obstacles");
    for(size_t i = 0; i < static_obst.size(); i++) {
      double rel_dist = static_obst[i].norm() - _uav_radius_;
      rel_dist = (rel_dist > 0.0) ? rel_dist : 0.01;

      double ctrl_vec_angle = (rel_dist > _col_vec_max_dist_ ? 0 : ((M_PI/2) / _col_vec_max_dist_) * rel_dist);
      double ctrl_vec_mag = (rel_dist  > _col_vec_max_dist_ ? 0 : _col_vec_const_ * (1 / sqrt(rel_dist) - 1 / sqrt(_col_vec_max_dist_)));
      e::Vector2d col_ctrl_vec = getSteerVec(pref_vec, static_obst[i], ctrl_vec_mag, ctrl_vec_angle);

      /* Viz info obst and control vectors//{*/
      geometry_msgs::Point vec_head, vec_tail;
      vec_tail.x = 0.0;
      vec_tail.y = 0.0;
      vec_tail.z = -_takeoff_height_;
      vec_head.x = col_ctrl_vec(0);
      vec_head.y = col_ctrl_vec(1);
      vec_head.z = -_takeoff_height_;

      viz_col_ctrl_vec.id++ ;
      viz_col_ctrl_vec.points.clear();
      viz_col_ctrl_vec.points.push_back(vec_tail);
      viz_col_ctrl_vec.points.push_back(vec_head);

      geometry_msgs::Point obst_pose;
      obst_pose.x = static_obst[i](0);
      obst_pose.y = static_obst[i](1);
      obst_pose.z = -_takeoff_height_;

      viz_vir_obst.points.push_back(vec_tail);
      viz_vir_obst.points.push_back(obst_pose);
      //}
      /* cout<<"Vir obst force vec: "<<vir_force(0)<<", "<<vir_force(1)<<" Norm: "<<vir_force.norm()<<endl; */
      msg_viz.markers.push_back(viz_col_ctrl_vec);

      net_vec += col_ctrl_vec;
    }
    //}

    if(static_obst.size() > 0.0) {
      net_vec /= double(static_obst.size());
    }

    if(!isfinite(net_vec(0)) || !isfinite(net_vec(1))) {
      net_vec = e::Vector2d{0, 0};
      ROS_WARN("[BoidsController] Collision control vector is not finite");
    }

    result.vector.x = net_vec(0);
    result.vector.y = net_vec(1);
    result.vector.z = 0.0;

    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.8;
    color.g = 0.8;
    color.b = 0.2;

    pubVecViz(result, pub_viz_collision_vec_, color);

    /* cout<<"No. of vir obsts: "<<double(msg_viz_vir_force.markers.size())<<endl; */
    pub_viz_col_vec_list_.publish(msg_viz);
    pub_viz_vir_obst_.publish(viz_vir_obst);

    return result;
  }

  //}

/* calcNavCtrlVec() //{ */
geometry_msgs::Vector3Stamped BoidsController::calcNavCtrlVec(swm_utils::IdStateStampedConstPtr self_state, swm_utils::IdStateArrayStampedConstPtr neighbor_states, nav_msgs::PathConstPtr des_path) {

  geometry_msgs::Vector3Stamped result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = _uav_frame_id_;
  result.vector.x = 0;
  result.vector.y = 0;
  result.vector.z = 0;

  e::Vector2d net_vec{0, 0};

  if(!des_path) {
    ROS_WARN("[BoidsController] No Path !");
    return result;
  }

  if (des_path->poses.empty()) {
    ROS_ERROR("[BoidsController] Path is empty!");
    return result;
  }

  /* calc nav vector //{ */
  auto nav_pt = getNextNavPoint(self_state, des_path);
  if(nav_pt) {

    /* tranform to required frame//{ */
    mrs_msgs::ReferenceStamped tmp = swm_r_utils::createReferenceStamped(nav_pt.value().header, swm_r_utils::createReference(nav_pt.value().point, 0.0));
    auto tf_pose = transformer_->transformSingle(tmp, result.header.frame_id);
    if(tf_pose) {
      net_vec = e::Vector2d{tf_pose.value().reference.position.x, tf_pose.value().reference.position.y};
    }
    else {
      ROS_ERROR("[BoidsController] Tf 'nav point' failed. From %s to %s", nav_pt.value().header.frame_id.c_str(), result.header.frame_id.c_str());
      return result;
    }//}

    /* cout<<"Current Nav vec X: "<<nav_vec(0)<<" Y: "<<nav_vec(1)<<endl; */
    net_vec = rescaleNavCtrlVec(net_vec, neighbor_states);
    /* cout<<"Rescaled Nav vec X: "<<nav_vec(0)<<" Y: "<<nav_vec(1)<<endl; */
  }
  else {
    net_vec = e::Vector2d{0, 0};
    ROS_WARN("[BoidsController] Reached the end of path, Only doing obstacle avoidance");
  }
  //}

if(!isfinite(net_vec(0)) || !isfinite(net_vec(1))) {
  net_vec = e::Vector2d{0, 0};
  ROS_WARN("[BoidsController] Nav control vector is not finite");
}
result.vector.x = net_vec(0);
result.vector.y = net_vec(1);

std_msgs::ColorRGBA color;
color.a = 1.0;
color.r = 0.1;
color.g = 0.7;
color.b = 0.1;

pubVecViz(result, pub_viz_nav_vec_, color);

return result;
}
//}

/* calcDirCtrlVec() //{ */
geometry_msgs::Vector3Stamped BoidsController::calcDirCtrlVec(int des_dir, double vec_mag) {

  geometry_msgs::Vector3Stamped result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = _uav_frame_id_;
  result.vector.x = 0;
  result.vector.y = 0;
  result.vector.z = 0;

  e::Vector2d net_vec{0, 0};

  switch (des_dir) {
    case 1 : {
               /* move forward */
               net_vec = e::Vector2d{vec_mag, 0};
               ROS_INFO_STREAM("[BoidsController] 'des_dir' is " << des_dir << " moving forward");
               break;
             }
    case 2 : {
               /* move left*/
               net_vec = e::Vector2d{0.0, vec_mag};
               ROS_INFO_STREAM("[BoidsController] 'des_dir' is " << des_dir << " moving left");
               break;
             }
    case 3 : {
               /* move backward*/
               net_vec = e::Vector2d{-vec_mag, 0};
               ROS_INFO_STREAM("[BoidsController] 'des_dir' is " << des_dir << " moving backward");
               break;
             }
    case 4 : {
               /* move right*/
               net_vec = e::Vector2d{0, -vec_mag};
               ROS_INFO_STREAM("[BoidsController] 'des_dir' is " << des_dir << " moving right");
               break;
             }
    default : net_vec = e::Vector2d{0.0, 0.0};
              ROS_INFO_STREAM("[BoidsController] 'des_dir' is " << des_dir << " staying");
              break;
  }

  if(!isfinite(net_vec(0)) || !isfinite(net_vec(1))) {
    net_vec = e::Vector2d{0, 0};
    ROS_WARN("[BoidsController] Direction control vector is not finite");
  }
  result.vector.x = net_vec(0);
  result.vector.y = net_vec(1);

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = 0.1;
  color.g = 0.7;
  color.b = 0.1;

  pubVecViz(result, pub_viz_dir_vec_, color);

  return result;
}
//}

/* calcFinalControlVec() //{ */
std::optional<geometry_msgs::Vector3Stamped> BoidsController::calcFinalControlVec(const geometry_msgs::Vector3Stamped prev_final_vec, int des_dir, std::shared_ptr<swm_ctrl::SwarmCommonDataHandler> common_data) {

  geometry_msgs::Vector3Stamped result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = prev_final_vec.header.frame_id;
  result.vector.x = 0;
  result.vector.y = 0;
  result.vector.z = 0;

  /* get data from manager //{ */
  swm_utils::IdStateStampedConstPtr self_state = common_data->getSelfState();
  if(!self_state) {
    ROS_ERROR("[BoidsController] Did not receive the self state");
    return nullopt;
  }

  swm_utils::IdStateArrayStampedConstPtr neighbor_states = common_data->getNeighborStates();
  if(!neighbor_states) {
    ROS_ERROR("[BoidsController] Did not receive any neighbor states");
    return nullopt;
  }
   //}

  e::Vector2d final_vec{0, 0};

  geometry_msgs::Vector3Stamped proximal_vec = calcProximalCtrlVec(neighbor_states, "quadexpo");
  if(proximal_vec.header.frame_id != result.header.frame_id) {
    proximal_vec.vector = swm_r_utils::createVector3(0, 0, 0);
    ROS_WARN("[BoidsController] Proximal Control Vec in frame: %s, required frame: %s", proximal_vec.header.frame_id.c_str(), result.header.frame_id.c_str());
  }
  ROS_INFO("[BoidsController] Proximal Control Vec: ( %f, %f)", proximal_vec.vector.x, proximal_vec.vector.y);

  /* geometry_msgs::Vector3Stamped nav_vec = calcNavCtrlVec(self_state, neighbor_states, des_path); */
  /* if(nav_vec.header.frame_id != result.header.frame_id) { */
  /*   nav_vec.vector = utils::createVector3(0, 0, 0); */
  /*   ROS_WARN("[BoidsController] Nav Control Vec in frame: %s, required frame: %s", nav_vec.header.frame_id.c_str(), result.header.frame_id.c_str()); */
  /* } */
  /* ROS_INFO("[BoidsController] Nav Control Vec: ( %f, %f)", nav_vec.vector.x, nav_vec.vector.y); */

  geometry_msgs::Vector3Stamped dir_vec = calcDirCtrlVec(des_dir, _dir_vel_mag_);

  if(dir_vec.header.frame_id != result.header.frame_id) {
    dir_vec.vector = swm_r_utils::createVector3(0, 0, 0);
    ROS_WARN("[BoidsController] Dir Control Vec in frame: %s, required frame: %s", dir_vec.header.frame_id.c_str(), result.header.frame_id.c_str());
  }
  ROS_INFO("[BoidsController] Dir Control Vec: ( %f, %f)", dir_vec.vector.x, dir_vec.vector.y);

  /* geometry_msgs::Vector3Stamped col_avoid_vec = calcCollisionCtrlVec(prev_final_vec, lidar_data); */
  /* if(col_avoid_vec.header.frame_id != result.header.frame_id) { */
  /*   col_avoid_vec.vector = utils::createVector3(0, 0, 0); */
  /*   ROS_WARN("[BoidsController] Collision Control Vec in frame: %s, required frame: %s", col_avoid_vec.header.frame_id.c_str(), result.header.frame_id.c_str()); */
  /* } */
  /* ROS_INFO("[BoidsController] Collision Control Vec: ( %f, %f)", col_avoid_vec.vector.x, col_avoid_vec.vector.y); */

  final_vec = _k_prox_ * e::Vector2d{proximal_vec.vector.x, proximal_vec.vector.y} + e::Vector2d{dir_vec.vector.x, dir_vec.vector.y}; 

  if(!isfinite(final_vec(0)) || !isfinite(final_vec(1))) {
    final_vec = e::Vector2d{0, 0};
    ROS_WARN("[BoidsController] Final Control Vec is not finite");
  }
  else {
    ROS_INFO("[BoidsController] Final Control Vec: ( %f, %f)", final_vec(0), final_vec(1));
  }

  cout<<endl;
  result.vector.x = final_vec(0);
  result.vector.y = final_vec(1);

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 0.0;

  pubVecViz(result, pub_viz_final_vec_, color);

  /* Info for mrs_uav_status//{ */
  string data;
  if(des_path_) {
    data = "UAVs: " + std::to_string(int(neighbor_states->states.size())) + " Mode: Informed";
  }
  else { 
    data = "UAVs: " + std::to_string(int(neighbor_states->states.size())) + " Mode: Uninformed";
  }

  std_msgs::String msg_mrs_uav_status;
  msg_mrs_uav_status.data = data;
  pub_mrs_uav_status_.publish(msg_mrs_uav_status);
  //}

  return result;
}
//}

/* calcHeading() //{ */
double BoidsController::calcHeading(string req_frame_id, swm_utils::IdStateArrayStampedConstPtr neighbor_states) {

  double heading = 0.0;

/* find avg angular direction in req_frame frame//{ */
 if(neighbor_states->states.size() > 0) {

    double avg_angle = 0.0;
    ROS_INFO("[BoidsController] Calc heading");
    for(auto itr = neighbor_states->states.begin(); itr != neighbor_states->states.end(); itr++) {
      e::Vector2d pose = e::Vector2d{itr->pose.position.x, itr->pose.position.y};

     /* transform to req_frame//{ */
      mrs_msgs::ReferenceStamped tmp = swm_r_utils::createReferenceStamped(neighbor_states->header, swm_r_utils::createReference(itr->pose.position, 0.0));
      auto tf_pose = transformer_->transformSingle(tmp, req_frame_id);
      if(tf_pose) {
        pose = e::Vector2d{tf_pose.value().reference.position.x, tf_pose.value().reference.position.y};
      }
      else {
        ROS_ERROR("[BoidsController] Tf 'neighbor_states' failed for 'calcHeading'. From %s to %s", neighbor_states->header.frame_id.c_str(), req_frame_id.c_str());
        return heading;
      }//}

      avg_angle += atan2(pose.normalized()(1), pose.normalized()(0));
    }
    heading = avg_angle/neighbor_states->states.size();
  }//}

  ROS_INFO_COND(neighbor_states->states.size() == 0, "[BoidsController] No UAVs visible, rotating for search");
  cout<<endl;

  if(!isfinite(heading)) {
    heading = 0.0;
    ROS_WARN("[BoidsController] Desired heading is not finite");
  }

  return heading;
}
//}

}  // namespace boids 

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(boids::BoidsController, swarm_control_manager::SwarmController);
