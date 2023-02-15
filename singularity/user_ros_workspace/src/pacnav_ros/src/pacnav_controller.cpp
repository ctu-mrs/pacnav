/* include header file of this class */
#include "pacnav_controller.h"

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace pacnav
{

  /* initialize() //{ */

  void PacnavController::initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& ros_name_space, std::shared_ptr<mrs_lib::Transformer> transformer) {

    // | ---------------- set my booleans to false ---------------- |
    // but remember, always set them to their default value in the header file
    // because, when you add new one later, you might forget to come back here
    got_goal_ = false;
    use_lidar_ = false;
    use_map_ = false;

    _name_ = name;

    /* obtain node handle */
    ros::NodeHandle nh(parent_nh, ros_name_space);
    /* global namespace handle for namespaces outside controller framework */
    ros::NodeHandle nh_g("/");

    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();

    // | ------------------- load ros parameters ------------------ |
    /* (mrs_lib implementation checks whether the parameter was loaded or not) */
    mrs_lib::ParamLoader pl_parent(parent_nh, "PacnavController");

    /* first load the params provided by the parent handle (swarm_control_manager) */
    pl_parent.loadParam("uav_name", _uav_name_);
    pl_parent.loadParam("uav_frame_id", _uav_frame_id_);
    pl_parent.loadParam("origin_frame_id", _origin_frame_id_);
    _max_vel_ = pl_parent.loadParam2<double>("vel_max");

    vector<string> req_self_data = pl_parent.loadParam2<vector<string>>("PacnavController/required_data/self_data");
    if (std::find(req_self_data.begin(), req_self_data.end(), "laser_scan") != req_self_data.end()) {
      use_lidar_ = true;
    }

    if (std::find(req_self_data.begin(), req_self_data.end(), "hector_map") != req_self_data.end()) {
      use_map_ = true;
    }

    /* load the params used for this controller */
    mrs_lib::ParamLoader pl_child(nh, "PacnavController");

    /* parameters for the UAV and simulation */
    _uav_radius_ = pl_child.loadParam2<double>("uav_radius");
    _takeoff_height_ = pl_child.loadParam2<double>("takeoff_height");
    _track_neighbors_  = pl_child.loadParam2<bool>("track_neighbors");
    _sigma_occl_ = pl_child.loadParam2<double>("sigma_occlusion");
    _lidar_range_ = pl_child.loadParam2<double>("lidar_range");

    /* parameters for the PACNav algorithm*/
    _obst_f_const_ = pl_child.loadParam2<double>("obst_f_const");
    _obst_f_max_dist_ = pl_child.loadParam2<double>("obst_f_max_dist");
    _k_nav_ = pl_child.loadParam2<double>("k_nav");
    _flock_safety_rad_ = pl_child.loadParam2<double>("flock_safety_rad");
    _invalidate_time_ = pl_child.loadParam2<double>("invalidate_time");
    _path_invalidate_time_ = pl_child.loadParam2<double>("path_invalidate_time");

    /* check if all parameters were loaded successfully */ 
    if (!pl_parent.loadedSuccessfully() || !pl_child.loadedSuccessfully())
    {
      /* if not, alert the user and shut the node down */ 
      ROS_ERROR("[PacnavController] parameter loading failure");
      ros::shutdown();
    }

    // | ------------------------ Tf ------------------------------ |

    transformer_ = transformer;

    // | ------------------ initialize subscribers ----------------- |

    if (use_map_) {

      mrs_lib::SubscribeHandlerOptions shopts(nh_g);
      shopts.node_name = name;
      shopts.threadsafe = true;
      shopts.no_message_timeout = ros::Duration(_invalidate_time_);
      shopts.use_thread_timer = false;

      mrs_lib::construct_object(sh_des_path_,
          shopts,
          pl_child.loadParam2<string>("path_in")
          );
    }

    // | ------------------ initialize publishers ----------------- |

    pub_viz_target_ = nh.advertise<geometry_msgs::PointStamped>("viz/target", 1);
    pub_viz_obst_force_ = nh.advertise<visualization_msgs::Marker>("viz/obst_force", 1);
    pub_viz_nav_vec_ = nh.advertise<visualization_msgs::Marker>("viz/nav_vec", 1);
    pub_viz_des_vel_ = nh.advertise<visualization_msgs::Marker>("viz/des_vel", 1);
    pub_viz_vir_obst_force_ = nh.advertise<visualization_msgs::MarkerArray>("viz/vir_obst_force", 1);
    pub_viz_vir_obst_ = nh.advertise<visualization_msgs::Marker>("viz/vir_obst", 1);
    pub_viz_neighbor_paths_ = nh.advertise<visualization_msgs::MarkerArray>("viz/neighbor_paths", 1);
    pub_viz_neighbor_states_ = nh.advertise<visualization_msgs::Marker>("viz/neighbor_states", 1);

    pub_mrs_uav_status_ = nh_g.advertise<std_msgs::String>(pl_child.loadParam2<string>("mrs_uav_status_out"), 1);

    // | --------------- initialize service servers --------------- |

    srv_set_goal_ = nh_g.advertiseService(pl_child.loadParam2<string>("set_goal"), &PacnavController::callbackSetGoal, this);

    // | --------------- initialize service clients --------------- |

    client_path_fndr_ = nh_g.serviceClient<mrs_msgs::Vec4>(pl_child.loadParam2<string>("set_path_fndr"));

    // | ---------- initialize private class varibales ------------------- |

    /* target should always be in origin frame because it is preserved over time and has low drift*/
    /* all uav target ids are geq 0 and non-uav target ids less than 0 */
    /* self id is -1 which is also the idle target */ 
    target_.id = -1;
    target_.header = swm_r_utils::createHeader(_origin_frame_id_, ros::Time::now());

    /* des velocity is easy to reason about in the uav frame */
    cur_des_vel_.header = swm_r_utils::createHeader(_uav_frame_id_, ros::Time::now());
    cur_des_vel_.vector = swm_r_utils::createVector3(0, 0, 0);

    is_init_ = true;
    ROS_INFO_ONCE("[PacnavController] Initialized");
  }
  //}

  /* activate() //{ */

  void PacnavController::activate() {

    is_active_ = true;
    ROS_INFO_STREAM("[PacnavController] Activated");
  }

  //}

  /* deactivate() //{ */

  void PacnavController::deactivate() {

    is_active_ = false;
    ROS_INFO_STREAM("[PacnavController] Deactivated");
  }

  //}

  /* update() //{ */

  std::optional<std::any> PacnavController::update(std::shared_ptr<swm_ctrl::SwarmCommonDataHandler> common_data) {

    if (!is_init_ || !is_active_) { 
      return nullopt;
    }

    /* Get data from swarm control manager //{ */

    /* self_state is fixed to be in origin frame from here on */
    swm_utils::IdStateStampedConstPtr rec_self_state = common_data->getSelfState();
    swm_utils::IdStateStamped self_state;

    if(!rec_self_state) {
      ROS_ERROR("[PacnavController] Did not receive self state");
      return nullopt;
    }
    else {

      /* transformation //{ */
      
      mrs_msgs::ReferenceStamped tmp = swm_r_utils::createReferenceStamped(rec_self_state->header, swm_r_utils::createReference(rec_self_state->state.pose.position, rec_self_state->state.heading));
      auto res = transformer_->transformSingle(tmp, _origin_frame_id_);
      if(res) {
        self_state.header = res.value().header;
        self_state.state.pose.position = res.value().reference.position;
        self_state.state.heading = res.value().reference.heading;
      }
      else {
        ROS_ERROR("[PacnavController] Tf 'self_state' failed. From %s to %s", rec_self_state->header.frame_id.c_str(), _origin_frame_id_.c_str());
        return nullopt;
      }
      
      //}
      
    }

    /* lidar data is fixed in a z-shifted uav frame, not easy to transform */
    /* can be used directly as x-y axis align with uav frame */
    sensor_msgs::LaserScanConstPtr lidar_data;

    if (use_lidar_) {

      lidar_data = common_data->getSelfLaserScan();

      if(!lidar_data) {
        ROS_ERROR("[PacnavController] Did not receive laser data");
        return nullopt;
      }
    }

    /* neighbor_states are fixed in uav frame as obst and nav forces are easy to work in this frame */
    swm_utils::IdStateArrayStampedConstPtr rec_neighbor_states = common_data->getNeighborStates();
    swm_utils::IdStateArrayStamped rel_neighbor_states;

    if(!rec_neighbor_states) {
      ROS_ERROR("[PacnavController] Did not receive any neighbor states");
      return nullopt;
    }
    else {

      updateNeighborStates(neighbor_states_, rec_neighbor_states, los_stamps_, lidar_data);

      /* paths should be stored in origin frame as it does not move with the uav */
      /* update path before tf */
      updateNeighborPaths(neighbor_paths_, neighbor_states_);

      rel_neighbor_states.header.frame_id = _uav_frame_id_;
      rel_neighbor_states.header.stamp = rec_neighbor_states->header.stamp;

      for(auto it = neighbor_states_.begin(); it != neighbor_states_.end(); it++) {

        /* transformation //{ */
        
        mrs_msgs::ReferenceStamped tmp = swm_r_utils::createReferenceStamped(rec_neighbor_states->header, swm_r_utils::createReference(it->second.point, 0.0));
        auto res = transformer_->transformSingle(tmp, _uav_frame_id_);
        if(res) {
          swm_utils::IdState tmp_state;
          tmp_state.id = it->first;
          tmp_state.pose.position = res.value().reference.position;
          tmp_state.heading = res.value().reference.heading;
        
          rel_neighbor_states.states.push_back(tmp_state);
        }
        else {
          ROS_ERROR("[PacnavController] Tf 'neighbor_states' failed. From %s to %s", rec_neighbor_states->header.frame_id.c_str(), _uav_frame_id_.c_str());
          return nullopt;
        }
        
        //}
        
      }
    }

    //}

    string status_data;

    /* select the target using path similarity and persistence or set it manually */
    if(got_goal_) {

      std::scoped_lock goal_lck(mutex_goal_);
      /* set received goal as new target, goal id is -2 */
      target_.header.stamp = ros::Time::now();
      target_.header.frame_id = goal_.header.frame_id;
      target_.id = -2;
      target_.point = goal_.point;

      ROS_INFO_THROTTLE(3, "[PacnavController] Received goal, I am an informed UAV");

      /* this status is shown by the MRS-status tab */
      status_data = "UAVs: " + std::to_string(int(rel_neighbor_states.states.size())) + ", T set by service";
    }
    else {
      ROS_INFO_THROTTLE(3, "[PacnavController] No goal from HUMAN, I am uninformed, searching for viable target");

      updateTarget(target_, self_state, neighbor_paths_);
      ROS_DEBUG_STREAM("[PacnavController] Target set. Going to UAV ID: " << target_.id);

      status_data  = "UAVs: " + std::to_string(int(rel_neighbor_states.states.size())) + ", T: " + std::to_string(target_.id);
    }

    pub_viz_target_.publish(swm_r_utils::createPointStamped(target_.header, target_.point));

    nav_msgs::Path des_path;

    /* get a path to the target using path finder (A* impl) */
    /* or just move to the target if no path finder or map is present */
    if (use_map_) {
      mrs_msgs::Vec4 msg_path_fndr;

      /* target must be in origin frame as path findr assumes the frame */
      if (target_.header.frame_id == _origin_frame_id_) {
        msg_path_fndr.request.goal[0] = target_.point.x;
        msg_path_fndr.request.goal[1] = target_.point.y;
        msg_path_fndr.request.goal[2] = _takeoff_height_;
        msg_path_fndr.request.goal[3] = 0.0;

        client_path_fndr_.call(msg_path_fndr);
        if(msg_path_fndr.response.success != true) {
          ROS_ERROR("[PacnavController] Path finder not responding, can not set current target as goal");
          return nullopt;
        }
        else {

          if(sh_des_path_.newMsg()){

            /* des path is also assumed to be in map origin frame */
            des_path = *sh_des_path_.getMsg();

            if (des_path.header.frame_id == _origin_frame_id_) {
              cur_des_vel_ = calcDesVel(cur_des_vel_, rel_neighbor_states, des_path, lidar_data);
            }
            else {
              ROS_ERROR("[PacnavController] 'des_path' needed in frame %s but currently is in %s", _origin_frame_id_.c_str(), des_path.header.frame_id.c_str());
              return nullopt;
            }
          }
        }
      }
      else {
        ROS_ERROR("[PacnavController] 'target' needed in frame %s but currently is in %s", _origin_frame_id_.c_str(), target_.header.frame_id.c_str());
        return nullopt;
      }
    }
    else {

      /* if no map is present, the uav can just move to the target without path planning */
      /* but will still use lidar for obst avoidance */

      /* des path is set to be in origin frame */
      des_path.header = swm_r_utils::createHeader(_origin_frame_id_, ros::Time::now());
      des_path.poses.push_back(swm_r_utils::createPoseStamped(self_state.header, self_state.state.pose));
      des_path.poses.push_back(swm_r_utils::createPoseStamped(target_.header, swm_r_utils::createPose(target_.point, geometry_msgs::Quaternion())));

      cur_des_vel_ = calcDesVel(cur_des_vel_, rel_neighbor_states, des_path, lidar_data);
    }

    /* check if the des velocity was updated, clean old cmd */
    if((ros::Time::now().toSec() - cur_des_vel_.header.stamp.toSec()) > _invalidate_time_ ) {
      cur_des_vel_.header.stamp = ros::Time::now();
      cur_des_vel_.vector = swm_r_utils::createVector3(0, 0, 0);
      ROS_WARN("[PacnavController] Desired velocity cmd is old, velocity set to ZERO");
    }

    /* publish info to mrs_uav_status node for easy debug */
    std_msgs::String msg_mrs_uav_status;
    msg_mrs_uav_status.data = status_data;
    pub_mrs_uav_status_.publish(msg_mrs_uav_status);

    /* prepare the cotnrol cmd for swarm_control_manager */
    mrs_msgs::VelocityReferenceStamped vel_cmd;
    vel_cmd.header = cur_des_vel_.header;
    vel_cmd.reference.velocity = cur_des_vel_.vector;

    if (_track_neighbors_ ) {

      /* this is the case when using UVDAR and cameras need to point to */
      /* capture max number of uavs in FOV */

      vel_cmd.reference.use_heading_rate = (rel_neighbor_states.states.size() > 0 ? false : true);
      vel_cmd.reference.use_heading = !vel_cmd.reference.use_heading_rate;

      vel_cmd.reference.heading = calcHeading(rel_neighbor_states);
      vel_cmd.reference.heading_rate = 0.5;
    }
    else {

      vel_cmd.reference.use_heading = true;
      vel_cmd.reference.use_heading_rate = false;

      /* when the target is set to self position */
      if (target_.id == -1) {

        vel_cmd.reference.heading = 0.0;
      }
      else {

        /* point heading to the target */
        mrs_msgs::ReferenceStamped tmp = swm_r_utils::createReferenceStamped(target_.header, swm_r_utils::createReference(target_.point, 0.0));
        auto res = transformer_->transformSingle(tmp, cur_des_vel_.header.frame_id);

        if(res) {

          e::Vector2d tf_target_vec{res.value().reference.position.x, res.value().reference.position.y};

          /* set heading to zero when very close to the target */
          /* prevents unnecessary rotations when UAVs are close */
          if(tf_target_vec.norm() > _uav_radius_/2.0) {

            vel_cmd.reference.heading = atan2(res.value().reference.position.y, res.value().reference.position.x);
          }
          else {

            vel_cmd.reference.heading = 0.0;
          }
        }
        else {
          ROS_ERROR("[PacnavController] Tf 'target' failed. From %s to %s", target_.header.frame_id.c_str(), cur_des_vel_.header.frame_id.c_str());
          return nullopt;
        }
      }
    }

    return std::any{vel_cmd};
  }

  //}

  // | ---------------------- callbacks --------------------- |

  /* /1* callbackPath() //{ *1/ */

  /* void PacnavController::callbackPath(mrs_lib::SubscribeHandler<nav_msgs::Path>& sh) { */

  /*   if (!is_init_) */
  /*     return; */

  /*   if (sh.getMsg()->header.frame_id == _origin_frame_id_) { */
  /*     got_path_ = true; */
  /*   } */
  /* } */

  /* //} */

  /* callbackSetGoal() //{ */
  bool PacnavController::callbackSetGoal(mrs_msgs::Vec4::Request &req, mrs_msgs::Vec4::Response &res) {

    if(!is_init_)
      return false;

    /* goal must be set in origin frame */
    /* because point in uav frame moves along with the uav */

    std::scoped_lock lck(mutex_goal_);
    goal_ = swm_r_utils::createPointStamped(swm_r_utils::createHeader(_origin_frame_id_, ros::Time::now()), swm_r_utils::createPoint(req.goal[0], req.goal[1], _takeoff_height_));
    got_goal_ = true;

    string msg = "In frame: " + _origin_frame_id_;
    res.message = msg;
    res.success = true;
    ROS_WARN_STREAM("[PacnavController] " << res.message);
    return true;
  }
  //}

  // | -------------------- functions ------------------- |

  /* calcObstForce() //{ */
  e::Vector2d PacnavController::calcObstForce(geometry_msgs::Vector3Stamped& prev_vel, swm_utils::IdStateArrayStamped& rel_neighbor_states, sensor_msgs::LaserScanConstPtr lidar_data) {

    e::Vector2d net_force{0, 0};
    e::Vector2d comp_vec{prev_vel.vector.x, prev_vel.vector.y};

    /* Viz init for info about obst and forces //{*/
    visualization_msgs::MarkerArray msg_viz;
    visualization_msgs::Marker viz_vir_obst_force;
    viz_vir_obst_force.header.stamp = ros::Time::now();
    viz_vir_obst_force.header.frame_id = prev_vel.header.frame_id;
    viz_vir_obst_force.ns = "Virtual Obstacle Forces";
    viz_vir_obst_force.id = 0;
    viz_vir_obst_force.type = visualization_msgs::Marker::ARROW;
    viz_vir_obst_force.action = visualization_msgs::Marker::ADD;
    viz_vir_obst_force.scale.x = 0.03;
    viz_vir_obst_force.scale.y = 0.06;
    viz_vir_obst_force.pose.orientation.w = 1.00;
    viz_vir_obst_force.color.a = 1.0; // Don't forget to set the alpha!
    viz_vir_obst_force.color.r = 0.0;
    viz_vir_obst_force.color.g = 0.2;
    viz_vir_obst_force.color.b = 0.4;

    visualization_msgs::Marker viz_vir_obst;
    viz_vir_obst.header.stamp = ros::Time::now();
    viz_vir_obst.header.frame_id = prev_vel.header.frame_id;
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

    vector<e::Vector2d> static_obst;
    if (use_lidar_) {
      vector<double> clean_lidar = cleanLidarData(lidar_data, _lidar_range_);
      static_obst = getObstLidar(clean_lidar, _obst_f_max_dist_ + _uav_radius_);
    }

    vector<e::Vector2d> dyn_obst;
    for(auto it = rel_neighbor_states.states.begin(); it != rel_neighbor_states.states.end(); it++) {
      e::Vector2d obst_pose{it->pose.position.x, it->pose.position.y};

      dyn_obst.push_back(obst_pose);
    }

    /* Obst force calc for static obst//{*/
    for(size_t i = 0; i < static_obst.size(); i++) {
      ROS_DEBUG_STREAM("[PacnavController] Calculating OBST FORCE for static obstacles");
      double rel_dist = static_obst[i].norm() - _uav_radius_;
      rel_dist = (rel_dist > 0.0) ? rel_dist : 0.01;

      double obst_force_dir = (rel_dist > _obst_f_max_dist_ ? 0 : ((M_PI/2) / _obst_f_max_dist_) * rel_dist);
      double obst_force_mag = (rel_dist  > _obst_f_max_dist_ ? 0 : _obst_f_const_ * (1 / sqrt(rel_dist) - 1 / sqrt(_obst_f_max_dist_)));
      e::Vector2d vir_force = getClosestVec(comp_vec, static_obst[i], obst_force_mag, obst_force_dir);

      /* Viz info obst and forces //{*/
      geometry_msgs::Point pt_s, pt_e;
      pt_s.x = 0.0;
      pt_s.y = 0.0;
      pt_s.z = -_takeoff_height_;
      pt_e.x = vir_force(0);
      pt_e.y = vir_force(1);
      pt_e.z = -_takeoff_height_;

      viz_vir_obst_force.id++ ;
      viz_vir_obst_force.points.clear();
      viz_vir_obst_force.points.push_back(pt_s);
      viz_vir_obst_force.points.push_back(pt_e);

      geometry_msgs::Point pt_obst;
      pt_obst.x = static_obst[i](0);
      pt_obst.y = static_obst[i](1);
      pt_obst.z = -_takeoff_height_;

      viz_vir_obst.points.push_back(pt_s);
      viz_vir_obst.points.push_back(pt_obst);
      //}
      /* cout<<"Vir obst force vec: "<<vir_force(0)<<", "<<vir_force(1)<<" Norm: "<<vir_force.norm()<<endl; */
      msg_viz.markers.push_back(viz_vir_obst_force);

      net_force += vir_force;
    }
    //}

    /* Obst force calc for dyn obst//{*/
    for(size_t i = 0; i < dyn_obst.size(); i++) {
      ROS_DEBUG_STREAM("[PacnavController] Calculating OBST FORCE for dynamic obstacles");
      double rel_dist = dyn_obst[i].norm() - _uav_radius_;
      rel_dist = (rel_dist > 0.0) ? rel_dist : 0.01;

      double obst_force_dir = (rel_dist > (_obst_f_max_dist_ + _uav_radius_) ? 0 : ((M_PI/2) / (_obst_f_max_dist_ + _uav_radius_)) * rel_dist);
      double obst_force_mag = (rel_dist  > (_obst_f_max_dist_ + _uav_radius_) ? 0 : _obst_f_const_ * (1 / sqrt(rel_dist) - 1 / sqrt(_obst_f_max_dist_ + _uav_radius_)));
      e::Vector2d vir_force = getClosestVec(comp_vec, dyn_obst[i], obst_force_mag, obst_force_dir);

      /* Viz info obst and forces //{*/
      geometry_msgs::Point pt_s, pt_e;
      pt_s.x = 0.0;
      pt_s.y = 0.0;
      pt_s.z = -_takeoff_height_;
      pt_e.x = vir_force(0);
      pt_e.y = vir_force(1);
      pt_e.z = -_takeoff_height_;

      viz_vir_obst_force.id++ ;
      viz_vir_obst_force.points.clear();
      viz_vir_obst_force.points.push_back(pt_s);
      viz_vir_obst_force.points.push_back(pt_e);

      geometry_msgs::Point pt_obst;
      pt_obst.x = dyn_obst[i](0);
      pt_obst.y = dyn_obst[i](1);
      pt_obst.z = -_takeoff_height_;

      viz_vir_obst.points.push_back(pt_s);
      viz_vir_obst.points.push_back(pt_obst);
      //}
      /* cout<<"Vir obst force vec: "<<vir_force(0)<<", "<<vir_force(1)<<" Norm: "<<vir_force.norm()<<endl; */
      msg_viz.markers.push_back(viz_vir_obst_force);

      net_force += vir_force;
    }
    //}

    if(static_obst.size() > 0) {
      net_force = net_force / static_obst.size();
    }

    if(dyn_obst.size() > 0) {
      net_force = net_force / dyn_obst.size();
    }

    if(!isfinite(net_force(0)) || !isfinite(net_force(1))) {
      net_force = e::Vector2d{0, 0};
      ROS_WARN("[PacnavController] Obst force is not finite");
    }

    geometry_msgs::Vector3Stamped viz_force = swm_r_utils::createVector3Stamped(swm_r_utils::createHeader(prev_vel.header.frame_id, ros::Time::now()), swm_r_utils::vector3FromEigen(e::Vector3d{net_force(0), net_force(1), 0}));

    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 0.8;
    color.g = 0.8;
    color.b = 0.2;

    pubVecViz(viz_force, pub_viz_obst_force_, color);

    pub_viz_vir_obst_force_.publish(msg_viz);
    pub_viz_vir_obst_.publish(viz_vir_obst);

    return net_force;
  }

  //}

  /* updateTarget() //{ */

  void PacnavController::updateTarget(pacnav::IdPointStamped& cur_target, swm_utils::IdStateStamped& self_state, map<int, vector<geometry_msgs::PointStamped>>& neighbor_paths) {

    /* stay at the same pose if no neighbors are observed */
    if(neighbor_paths.size() == 0) {

      cur_target.id = -1;
      cur_target.point = self_state.state.pose.position;
      ROS_WARN_THROTTLE(2, "[PacnavController] No UAVs visible, target set to self pose");
    }
    else {

      /* update the previous target or init in case it is invalid now */
      if(cur_target.id == -1) {
        cur_target.point = self_state.state.pose.position;
        ROS_WARN_THROTTLE(2, "[PacnavController] Updating self pose target, no UAV chosen yet");
      }
      else {

        /* first update the already tracked target */
        if(!neighbor_paths[cur_target.id].empty()) {
          cur_target.point = neighbor_paths[cur_target.id].back().point;
          ROS_DEBUG_STREAM("[PacnavController] Updating position of cur_target, ID: " << cur_target.id);
        }
        else {
          /* re-init when the existing target has no path */
          cur_target.id = -1;
          cur_target.point = self_state.state.pose.position;
          ROS_WARN_THROTTLE(2, "[PacnavController] Target path was empty, target set to self pose");
        }
      }

      /* select a new target based on path similarity metric */
      map<int, map<int, float>> similarity_mat;
      similarity_mat = getSimilarityMat(neighbor_paths);

      /* try to find a better target */ 
      for(auto it = neighbor_paths.begin(); it != neighbor_paths.end(); ++it) {

        ROS_DEBUG_STREAM("[PacnavController] Checking cand id: " << it->first);

        if(!it->second.empty() && it->first != cur_target.id) {

          ROS_DEBUG_STREAM("[PacnavController] Cand has finite path len, checking relative dist");

          geometry_msgs::PointStamped cand_pt = it->second.back();
          float cand_dist = (swm_r_utils::pointToEigen(cand_pt.point) - swm_r_utils::pointToEigen(self_state.state.pose.position)).norm();

          /* only neighbors outside flock radius are feasible targets(avoids jumping of target when agents are close) */
          if(cand_dist > _flock_safety_rad_) {

            ROS_DEBUG_STREAM("[PacnavController] Cand is outside flocking radius, checking for convergence");

            pacnav::IdPointStamped cand_target;
            cand_target.id = it->first;
            cand_target.header = cand_pt.header;
            cand_target.point = cand_pt.point;

            /* initialize the target */
            if(cur_target.id == -1) {
              cur_target = cand_target;
              ROS_DEBUG_STREAM("[PacnavController] No initial target was set, setting cand as target");
            }
            else {

              /* check if cand is converging, non-converging cand is more likely to know the goal */
              if(!isCandConverging(cur_target.point, neighbor_paths[cand_target.id])) {

                ROS_DEBUG_STREAM("[PacnavController] Cand is not converging to the curren target, checking for similarity score");

                double cand_total_score = 0;
                double target_total_score = 0;

                for(auto it2 = neighbor_paths.begin(); it2 != neighbor_paths.end(); ++it2) {
                  cand_total_score += similarity_mat[cand_target.id][it2->first];
                  target_total_score += similarity_mat[cur_target.id][it2->first];
                }
                ROS_DEBUG_STREAM("[PacnavController] Cand similarity score: " << cand_total_score << ", Cur Target similarity score: " << target_total_score);

                /* choose cand if it has higher similarity than cur target */
                if(cand_total_score > target_total_score) {
                  ROS_DEBUG_STREAM("[PacnavController] Cand has higher similarity score, switching target to id: " << cand_target.id);
                  cur_target = cand_target;
                }
                else {
                  ROS_DEBUG_STREAM("[PacnavController] Cand has lower similarity score, skipping");
                }
              }
              else {
                ROS_DEBUG_STREAM("[PacnavController] Cand is converging, skipping");
              }
            }
          }
          else {
            ROS_DEBUG_STREAM("[PacnavController] Cand inside flock radius, skipping");
          }
        }
        else {
          ROS_DEBUG_STREAM("[PacnavController] Cand has empty path or is already a target, skipping");
        }
      }
    }

    /* update the timestamp (imp. for the case when target point is not updated) */
    cur_target.header.frame_id = _origin_frame_id_;
    cur_target.header.stamp = ros::Time::now();

    geometry_msgs::PointStamped viz_target = swm_r_utils::createPointStamped(cur_target.header, cur_target.point);
    pub_viz_target_.publish(viz_target);
  }

  //}

  /* calcDesVel() //{ */
  geometry_msgs::Vector3Stamped PacnavController::calcDesVel(geometry_msgs::Vector3Stamped& prev_vel, swm_utils::IdStateArrayStamped& rel_neighbor_states, nav_msgs::Path& des_path, sensor_msgs::LaserScanConstPtr lidar_data) {

    geometry_msgs::Vector3Stamped result = swm_r_utils::createVector3Stamped(swm_r_utils::createHeader(prev_vel.header.frame_id, ros::Time::now()), swm_r_utils::createVector3(0, 0, 0));

    e::Vector2d cur_vel_vec{0, 0}, nav_vec{0, 0}, obst_avoid_vec{0, 0};

    if (des_path.poses.empty()) {

      ROS_WARN("[PacnavController] Path is empty!");
      return result;
    }
    else {

      /* calc nav vector and publish its viz//{ */
      nav_vec = getNextNavPoint(des_path);
      nav_vec = rescaleNavVec(nav_vec, rel_neighbor_states, _flock_safety_rad_, got_goal_);

      ROS_DEBUG_STREAM("[PacnavController] Nav vec X: " << nav_vec(0) << ", Y: " << nav_vec(1) << ", norm: " << nav_vec.norm());

      geometry_msgs::Vector3Stamped viz_nav_vec = swm_r_utils::createVector3Stamped(swm_r_utils::createHeader(result.header.frame_id, ros::Time::now()), swm_r_utils::createVector3(nav_vec(0), nav_vec(1), 0));

      std_msgs::ColorRGBA color;
      color.a = 1.0;
      color.r = 0.0;
      color.g = 0.7;
      color.b = 0.1;

      pubVecViz(viz_nav_vec, pub_viz_nav_vec_, color);

      //}

      /* calc obstacle avoidance vector//{ */

      obst_avoid_vec = calcObstForce(prev_vel, rel_neighbor_states, lidar_data);
      ROS_DEBUG_STREAM("[PacnavController] Obst vec X: " << obst_avoid_vec(0) << ", Y: " << obst_avoid_vec(1) << ", norm: " << obst_avoid_vec.norm());

      //}

      cur_vel_vec = _k_nav_ * nav_vec + obst_avoid_vec;

      if (cur_vel_vec.norm() > _max_vel_) {
        cur_vel_vec = _max_vel_ * cur_vel_vec.normalized();
      }

      if(!isfinite(cur_vel_vec(0)) || !isfinite(cur_vel_vec(1))) {

        cur_vel_vec = e::Vector2d{0, 0};
        ROS_WARN("[PacnavController] Desired velocity is not finite");
      }

      result.vector.x = cur_vel_vec(0);
      result.vector.y = cur_vel_vec(1);
      result.vector.z = 0.0;
      color.a = 1.0;
      color.r = 0.0;
      color.g = 0.0;
      color.b = 0.0;
      pubVecViz(result, pub_viz_des_vel_, color);

      ROS_DEBUG("[PacnavController] Desired Velocty is X: %f, Y: %f, Z: %f", result.vector.x, result.vector.y, result.vector.z);
      return result;
    }
  }

  //}

  /* calcHeading() //{ */

  double PacnavController::calcHeading(swm_utils::IdStateArrayStamped& rel_neighbor_states) {

    double heading = 0.0;

    if(rel_neighbor_states.states.size() > 0) {

      double sum_angle = 0.0;
      ROS_DEBUG("[PacnavController] Calc heading");

      /* simply use avg angle to currently observed neighbors */
      /* it will make sure that most of them remian in the view */
      for(auto it = rel_neighbor_states.states.begin(); it != rel_neighbor_states.states.end(); it++) {

        sum_angle += atan2(it->pose.position.y, it->pose.position.x);
      }

      heading = sum_angle/rel_neighbor_states.states.size();
    }

    ROS_DEBUG_COND(rel_neighbor_states.states.size() == 0, "[PacnavController] No UAVs visible, will rotate for search");

    /* just a sanity check */
    if(!isfinite(heading)) {
      heading = 0.0;
      ROS_WARN("[PacnavController] Desired heading is not finite");
    }

    return heading;
  }

  //}

  /* | -------------------- vector utils ------------------- | //{*/

  /* pubVecViz() //{ */ 
  void PacnavController::pubVecViz(geometry_msgs::Vector3Stamped& force, ros::Publisher& pub, std_msgs::ColorRGBA color) {

    visualization_msgs::Marker viz;

    viz.header = force.header;
    viz.ns = "Virtual Force Vector";
    viz.id = 0;
    viz.type = visualization_msgs::Marker::ARROW;
    viz.action = visualization_msgs::Marker::ADD;
    viz.scale.x = 0.04;
    viz.scale.y = 0.07;
    viz.color = color;
    viz.pose.orientation.w = 1.0;

    geometry_msgs::Point beg_pt = swm_r_utils::createPoint(0, 0 , -_takeoff_height_);
    geometry_msgs::Point end_pt = swm_r_utils::createPoint(force.vector.x, force.vector.y, -_takeoff_height_);

    viz.points.push_back(beg_pt);
    viz.points.push_back(end_pt);

    pub.publish(viz);
  }

  //}

  /* //}*/

  /* | -------------------- navigation force utils ------------------- | //{*/

  /* getNextNavPoint() //{ */
  e::Vector2d PacnavController::getNextNavPoint(nav_msgs::Path& des_path) {

    /* the nav pt is in uav frame and the dist to the pt is directly */ 
    /* used as the ctrl vec for navigation */
    e::Vector2d nav_pt{0, 0};

    for(size_t i = 1; i < des_path.poses.size(); i++) {

      mrs_msgs::ReferenceStamped tmp = swm_r_utils::createReferenceStamped(des_path.poses[i].header, swm_r_utils::createReference(des_path.poses[i].pose.position, 0.0));
      auto tf_pose = transformer_->transformSingle(tmp, _uav_frame_id_);
      if(tf_pose) {

        e::Vector2d cand_pt{tf_pose.value().reference.position.x, tf_pose.value().reference.position.y};

        if (cand_pt.norm() > (_uav_radius_/4.0)) {

          /* return the first pt that is farther than radius/4 */
          nav_pt = cand_pt;
          break;
        }
      }
      else {

        ROS_ERROR("[PacnavController] Tf 'des_path' failed. From %s to %s", des_path.header.frame_id.c_str(), _uav_frame_id_.c_str());

        /* must return zero vec when the path can not be tranformed */
        /* this might be due to absence of tf tree so its best to avoid making any decisions now */
        break;
      }
    }

    return nav_pt;
  }

  //}

  /* rescaleNavVec() //{ */
  e::Vector2d PacnavController::rescaleNavVec(e::Vector2d& nav_vec, swm_utils::IdStateArrayStamped& rel_neighbor_states, double flock_rad, bool got_goal) {

    e::Vector2d res = nav_vec;

    if(got_goal) {

      double avg_dist = 0;
      for(auto it = rel_neighbor_states.states.begin(); it != rel_neighbor_states.states.end(); it++) {

        e::Vector2d pose = e::Vector2d{it->pose.position.x, it->pose.position.y};

        double dist = pose.norm() - _uav_radius_;
        dist = (dist > 0.0) ? dist : 0.01;

        avg_dist += dist;
      }

      if(rel_neighbor_states.states.size() > 0) {
        avg_dist /= rel_neighbor_states.states.size();
      }

      double sf = (avg_dist - 2.0*flock_rad) > 0.0 ? 0.3 : (-(1.0/pow(2*(flock_rad + _uav_radius_), 2.0))*pow(avg_dist, 2.0) + 1.0);
      res = sf * res;

      ROS_DEBUG("[PacnavController] Slowing down for others to catch up, rescaling NAV_VEC by factor: %f", sf);
    }
    else {

      /* rescale the nav vec to avoid collision with nearby agents */
      /* the vec is iteratively scaled in the component para to */ 
      /* the rel vec towards a nearby agent */
      for(auto it = rel_neighbor_states.states.begin(); it != rel_neighbor_states.states.end(); it++) {

        e::Vector2d rel_vec = e::Vector2d{it->pose.position.x, it->pose.position.y};
        double proj = res.dot(rel_vec.normalized());

        if(proj > 0.0) {
            
          double dist = rel_vec.norm() - _uav_radius_;
          dist = (dist > 0.0) ? dist : 0.01;

          double sf = (dist - flock_rad) > 0.0 ? 1.0 : (1.0/pow(flock_rad, 8.0))*pow(dist, 8.0);

          /* keep the ortho vec */
          e::Vector2d ortho_vec = res - (proj * rel_vec.normalized());

          /* rescale the para vec down to decrease vel in the direction of neighbor */
          e::Vector2d para_vec = sf * proj * rel_vec.normalized();

          res = ortho_vec + para_vec;

          ROS_DEBUG_STREAM("[PacnavController] Approaching close to another Agent: "<<it->id<<", rescaling NAV_VEC by factor: "<<sf);
        }
      }
    }

    return res;
  }
  //}

  //}

  /* | -------------------- neighbor info utils ------------------- | //{*/

  /* updateNeighborStates()//{ */
  void PacnavController::updateNeighborStates(map<int, geometry_msgs::PointStamped>& neighbor_states, swm_utils::IdStateArrayStampedConstPtr rec_neighbor_states, map<int, ros::Time>& los_stamps, sensor_msgs::LaserScanConstPtr lidar_data) {

    /* Viz init for neighbor states //{*/
    visualization_msgs::Marker viz_n_states;
    viz_n_states.header.frame_id = rec_neighbor_states->header.frame_id;
    viz_n_states.header.stamp = ros::Time::now();
    viz_n_states.ns = "neighbor states";
    viz_n_states.id = 0;
    viz_n_states.type = visualization_msgs::Marker::SPHERE_LIST;
    viz_n_states.action = visualization_msgs::Marker::ADD;
    viz_n_states.scale.x = 0.05;
    viz_n_states.scale.y = 0.05;
    viz_n_states.scale.z = 0.05;
    viz_n_states.pose.orientation.w = 1.0;
    viz_n_states.color.a = 1.0; // Don't forget to set the alpha!
    viz_n_states.color.r = 1.0;
    viz_n_states.color.g = 0.0;
    viz_n_states.color.b = 0.0;
    //}

    /* the neighbor states are updated with the noise model and checked for occlusion if uvdar is being used for tracking */
    if(_track_neighbors_  && lidar_data) {
      applyOcllusion(neighbor_states, rec_neighbor_states, los_stamps_, cleanLidarData(lidar_data, _lidar_range_));
    }
    else {
      /* update states from recieved states if GPS poses are shared */
      for(auto it = rec_neighbor_states->states.begin(); it != rec_neighbor_states->states.end(); it++) {
        neighbor_states[it->id] = swm_r_utils::createPointStamped(rec_neighbor_states->header, it->pose.position);
      }
    }

    for(auto it = neighbor_states.begin(); it != neighbor_states.end(); ++it){
      viz_n_states.points.push_back(it->second.point);
    }

    pub_viz_neighbor_states_.publish(viz_n_states);
  }
  //}

  /* updateNeighborPaths()//{ */
  void PacnavController::updateNeighborPaths(map<int, vector<geometry_msgs::PointStamped>>& neighbor_paths, map<int, geometry_msgs::PointStamped>& neighbor_states) {

    /* Viz init for neighbor paths vec//{*/
    visualization_msgs::MarkerArray msg_viz_neighbor_paths;
    visualization_msgs::Marker viz_neighbor_paths;
    viz_neighbor_paths.header.frame_id = _origin_frame_id_;
    viz_neighbor_paths.header.stamp = ros::Time::now();
    viz_neighbor_paths.ns = "neighbor paths";
    viz_neighbor_paths.id = 0;
    viz_neighbor_paths.type = visualization_msgs::Marker::LINE_STRIP;
    viz_neighbor_paths.action = visualization_msgs::Marker::ADD;
    viz_neighbor_paths.scale.x = 0.05;
    viz_neighbor_paths.pose.orientation.w = 1.0;
    viz_neighbor_paths.color.a = 1.0; // Don't forget to set the alpha!
    viz_neighbor_paths.color.r = 0.2;
    //}

    /* update paths from recieved states */
    for(auto it = neighbor_states.begin(); it != neighbor_states.end(); it++) {
      /* if(neighbor_paths[cur_id].size() < _neighbor_path_len_) { */
      /*   neighbor_paths[cur_id].push_back(utils::createPointStamped(rec_neighbor_states->header, it->pose.position)); */
      /* } */
      /* else { */
      /*   neighbor_paths[cur_id].erase(neighbor_paths[cur_id].cbegin()); */
        neighbor_paths[it->first].push_back(swm_r_utils::createPointStamped(it->second.header, it->second.point));
      /* } */
    }

    /* empty paths should be removed as they have no useful info */
    vector<int> empty_path_ids;

    /* clean old path points and publish the neighbor_paths */
    for(auto itr = neighbor_paths.begin(); itr != neighbor_paths.end(); ++itr){

      if(!itr->second.empty()) {
        /* clean old path points */
        auto oldest_pt = itr->second.begin();
        if((ros::Time::now().toSec() - oldest_pt->header.stamp.toSec()) > _path_invalidate_time_ ) {
          itr->second.erase(oldest_pt);
        }

        for(int j = 0; j < itr->second.size(); ++j){
          viz_neighbor_paths.points.push_back(itr->second[j].point);
        }

        viz_neighbor_paths.id = itr->first;
        msg_viz_neighbor_paths.markers.push_back(viz_neighbor_paths);

        viz_neighbor_paths.points.clear();
      }
      else {
        empty_path_ids.push_back(itr->first);
      }
    }

    /* erase the paths that are empty, its needed when target is updated */
    for (int i = 0; i < empty_path_ids.size(); i++) {
      neighbor_paths.erase(empty_path_ids[i]);
    }

    pub_viz_neighbor_paths_.publish(msg_viz_neighbor_paths);
  }
  //}

  /* applyOcllusion() //{ */

  void PacnavController::applyOcllusion(map<int, geometry_msgs::PointStamped>& neighbor_states, swm_utils::IdStateArrayStampedConstPtr rec_neighbor_states, map<int, ros::Time>& los_stamps, vector<double> lidar_data) {

    e::Matrix<double, 3, 3> std_dev;
    std_dev <<
      _sigma_occl_, 0, 0,
      0, _sigma_occl_, 0,
      0, 0, _sigma_occl_;

    for(auto it = rec_neighbor_states->states.begin(); it != rec_neighbor_states->states.end(); it++) {

      bool los = neighborInLOS(rec_neighbor_states->header, it->pose.position, lidar_data);

      if(los) {

        /* cout << "Neighbor " << it->id << " is in LOS, adding noise" << endl; */

        e::Vector3d tmp_p = swm_r_utils::pointToEigen(it->pose.position);
        e::Vector3d sampled_p = swm_m_utils::sampleRandVec(tmp_p, std_dev);
        neighbor_states[it->id] = swm_r_utils::createPointStamped(rec_neighbor_states->header, swm_r_utils::pointFromEigen(sampled_p));
        los_stamps[it->id] = rec_neighbor_states->header.stamp;

        /* cout << "position error: " << (tmp_p - sampled_p).norm() << endl; */
      }
      else {
        if((ros::Time::now() - los_stamps[it->id]).toSec() <= _invalidate_time_) {

          /* cout << "Neighbor " << it->id << " not in LOS, adding noise to prev measure" << endl; */

          e::Vector3d tmp_p = swm_r_utils::pointToEigen(neighbor_states[it->id].point);
          e::Vector3d sampled_p = swm_m_utils::sampleRandVec(tmp_p, std_dev);
          neighbor_states[it->id] = swm_r_utils::createPointStamped(rec_neighbor_states->header, swm_r_utils::pointFromEigen(sampled_p));

          /* cout << "position error: " << (tmp_p - sampled_p).norm() << endl; */
        }
        else {

          /* cout << "Neighbor " << it->id << " not in LOS for " << _invalidate_time_ << " seconds, deleting it" << endl; */

          neighbor_states.erase(it->id);
        }
      }
    }
  }

  //}

  /* neighborInLOS() //{ */

  bool PacnavController::neighborInLOS(const std_msgs::Header& header, const geometry_msgs::Point& position, vector<double>& lidar_data) {

    bool los = false;
    int n_idx;

    mrs_msgs::ReferenceStamped tmp = swm_r_utils::createReferenceStamped(header, swm_r_utils::createReference(position, 0.0));
    auto res = transformer_->transformSingle(tmp, _uav_frame_id_);
    if(res) {
      n_idx = angle2Index(std::atan2(res.value().reference.position.y, res.value().reference.position.x), lidar_data.size());
    }
    else {
      ROS_ERROR("[PacnavController] Tf neighborInLOS failed. From %s to %s", header.frame_id.c_str(), _uav_frame_id_.c_str());
    }

    if(lidar_data[n_idx] > swm_r_utils::pointToEigen(res.value().reference.position).norm()) {
      los = true;
    }

    /* cout << "LOS: " << los << "lidar range: " << lidar_data[n_idx] << endl; */

    return los;
  }

  //}

  //}

  // | -------------------- non-member functions ------------------- |

  /* | -------------------- target selection utils  ------------------- | //{*/

  /* isCandConverging() //{ */
  /**
   * @brief : checks if a cand UAV is converging to the current target.
   * The check returns true if the candidate path is moving towards the target.
   *
   * @param target
   * @param cand_path
   *
   * @return 
   */
  bool isCandConverging(geometry_msgs::Point& target, vector<geometry_msgs::PointStamped>& cand_path) {

    e::Vector3d target_pt = swm_r_utils::pointToEigen(target);
    float init_dist = (swm_r_utils::pointToEigen(cand_path.begin()->point) - target_pt).norm();
    float final_dist = (swm_r_utils::pointToEigen(cand_path.rbegin()->point) - target_pt).norm();

    if(final_dist < init_dist) {
      return true;
    }
    else { 
      return false;
    }
  }
  //}

/* TODO: remove sz conditions and make the mat undefined for sz <= 2 */
  /* getSimilarityMat()//{ */
  map<int, map<int, float>> getSimilarityMat(map<int, vector<geometry_msgs::PointStamped>>& neighbor_paths) {

    map<int, map<int, float>> similarity_mat;

    for(auto it = neighbor_paths.begin(); it != neighbor_paths.end(); ++it) {
      int base_id = it->first;
      vector<geometry_msgs::PointStamped> base_path = neighbor_paths[base_id];

      for(auto it2 = neighbor_paths.begin(); it2 != neighbor_paths.end(); ++it2) {
        int cand_id = it2->first;
        vector<geometry_msgs::PointStamped> cand_path = neighbor_paths[cand_id];

        int sz = std::min(base_path.size(), cand_path.size());

        if(base_id == cand_id){

          /* encoding persistence by comparing similarity of initial path vec */
          /* with the rest of the path */
          if(sz - 2 <= 0) {
            similarity_mat[base_id][cand_id] = -1.0;
          }
          else {
            float sum_score = 0;
            for(int k = 0; k < sz - 2; ++k) {
              e::Vector3d vec1 = swm_r_utils::pointToEigen(base_path[k + 2].point) - swm_r_utils::pointToEigen(base_path[k + 1].point);
              e::Vector3d vec2 = swm_r_utils::pointToEigen(base_path[k + 1].point) - swm_r_utils::pointToEigen(base_path[k].point);

              sum_score += vec1.normalized().dot(vec2.normalized());
            }

            /* normalize similarity score to range [-1, 1] */
            /* negative values useful as dot product will be zero at PI/2 */
            similarity_mat[base_id][base_id] = sum_score / float(sz - 2);
          }
        }
        else{

          if(sz - 1 <= 0) {
            similarity_mat[base_id][cand_id] = -1.0;
          }
          else {
            float sum_score = 0;
            for(int k = 0; k < sz - 1; ++k) {
              e::Vector3d vec1 = swm_r_utils::pointToEigen(cand_path[k + 1].point) - swm_r_utils::pointToEigen(cand_path[k].point);
              e::Vector3d vec2 = swm_r_utils::pointToEigen(base_path[k + 1].point) - swm_r_utils::pointToEigen(base_path[k].point);

              sum_score += vec1.normalized().dot(vec2.normalized());
            }

            /* normalize similarity score to range [-1, 1] */
            /* negative values useful as dot product will be zero at PI/2 */
            similarity_mat[base_id][cand_id] = sum_score / float(sz - 1);
          }
        }
      }

    }

    return similarity_mat;
  }
  //}

  //}

  /* | -------------------- obst force utils ------------------- | //{*/

  /* angle2Index() //{ */
  int angle2Index(double angle, int max_index) {

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
  double index2Angle(int index, int max_index) {

    double angle = 0.0;

    if(index <= max_index) {
      angle = double((2 * M_PI * index)/max_index);
    }
    else {
      ROS_ERROR("[PacnavController] Index out of range for angle conversion");
    }

    return angle;
  }

  //}

  /* cleanLidarData()  //{ */
  vector<double> cleanLidarData(sensor_msgs::LaserScanConstPtr lidar_data, double max_range){
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

  /* getObstLidar()  //{ */
  vector<e::Vector2d> getObstLidar(vector<double>& lidar_ranges, double max_range){

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

  /* getClosestVec()  //{ */
  e::Vector2d getClosestVec(e::Vector2d& comp_vec, e::Vector2d& base_vec, double des_mag, double des_dir){
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

}  // namespace pacnav

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(pacnav::PacnavController, swarm_control_manager::SwarmController)
