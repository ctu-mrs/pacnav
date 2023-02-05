#include <cmath>
#include <mrs_msgs/Path.h>
#include <mrs_msgs/VelocityReferenceStamped.h>

#include <Eigen/Geometry>
#include <swarm_control_manager/swarm_controller.h>
#include <swarm_control_manager/common_data_handler.h>
#include <swarm_utils/ros_utils.h>
#include <swarm_utils/IdStateStamped.h>

#include <pluginlib/class_list_macros.h>

using namespace std;
namespace e = Eigen;
namespace swm_ctrl_ctrl = swarm_control_manager;
namespace swm_utils = swarm_utils;
namespace swm_r_utils = swarm_utils::ros_utils;

namespace test_controller{

  class TestController: public swm_ctrl::SwarmController{
    string _name_;
    string _uav_name_;
    string _uav_frame_id_;
    string _origin_frame_id_;

    string _test_mode_;
    float _polygon_side_;
    double _traj_gen_time_;

    std::shared_ptr<mrs_lib::Transformer> transformer_;

    bool is_initialized_ = false;
    bool is_active_ = false;

    bool test_init_ = false;
    vector<e::Vector3d> desired_points_;
    int cur_point_idx_ = 0;
    e::Vector2d circle_center_;
    ros::Publisher pub_translated_map_;

    public:
    void initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& ros_name_space, std::shared_ptr<mrs_lib::Transformer> transformer) override;
    void activate() override;
    void deactivate() override;
    std::optional<std::any> update(std::shared_ptr<swm_ctrl::SwarmCommonDataHandler> common_data) override;
  };

  /* initialize() //{ */

  void TestController::initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& ros_name_space, std::shared_ptr<mrs_lib::Transformer> transformer) {
    ros::NodeHandle nh(parent_nh, ros_name_space);
    _name_ = name;
    transformer_ = transformer;
    ros::Time::waitForValid();

    test_init_ = false;

    // Load parent parameters
    mrs_lib::ParamLoader pl_parent(parent_nh, "TestController");

    pl_parent.loadParam("uav_name", _uav_name_);
    pl_parent.loadParam("uav_frame_id", _uav_frame_id_);
    pl_parent.loadParam("origin_frame_id", _origin_frame_id_);
    pl_parent.loadParam("traj_gen_time", _traj_gen_time_);
    pl_parent.loadParam("TestController/control_type", _test_mode_);

    mrs_lib::ParamLoader pl_child(nh, "TestController");
    _polygon_side_ = pl_child.loadParam2<float>("polygon_side");

    if (!pl_parent.loadedSuccessfully() || !pl_child.loadedSuccessfully()) {
      ROS_ERROR_STREAM("[TestController]: Unable to laod parameters, check config and launch files");
      ros::shutdown();
    }

    pub_translated_map_ = nh.advertise<nav_msgs::OccupancyGrid>("translated_map", 1);

    ROS_DEBUG_STREAM("[TestController]: Initialized");
    is_initialized_ = true;
  }

  //}

  /* activate() //{ */

  void TestController::activate() {
    is_active_ = true;
    ROS_DEBUG_STREAM("[TestController]: Activated");
  }

  //}

  /* deactivate() //{ */

  void TestController::deactivate() {
    is_active_ = false;
    ROS_DEBUG_STREAM("[TestController]: Deactivated");
  }

  //}

  /* getTriangle() //{ */

  vector<e::Vector3d> getTriangle(e::Vector3d& init_pose, float side_len){
    vector<e::Vector3d> triangle_coord;
    triangle_coord.push_back(init_pose);

    triangle_coord.push_back(init_pose + e::Vector3d(side_len/2.0, side_len/std::sqrt(2), side_len/std::sqrt(2)));
    triangle_coord.push_back(init_pose + e::Vector3d(side_len, 0.0, 0.0));

    return triangle_coord;
  }

  //}

  /* update() //{ */

  std::optional<std::any> TestController::update(std::shared_ptr<swm_ctrl::SwarmCommonDataHandler> common_data) {
    if (!is_active_ || !is_initialized_) return std::nullopt;

    swm_utils::IdStateStampedConstPtr self_state = common_data->getSelfState();

    // Self state
    if(self_state) {
      ROS_DEBUG_STREAM("[TestController]: Received self state.\n" <<
          "UAV: " << self_state->state.id << "\n" <<
          "Pose: (" << self_state->state.pose.position.x << ", " << self_state->state.pose.position.y << ", " << self_state->state.pose.position.z << ")\n" <<
          "Heading: " << self_state->state.heading);
    }
    else {
      ROS_ERROR("[TestController]: Did not receive the self state");
    }

    // Hector map
    /* nav_msgs::OccupancyGridConstPtr self_hector_map = common_data->getSelfHectorMap(); */
    /* if(self_hector_map) { */
    /*   ROS_DEBUG_STREAM("[TestController]: Received self hector map.\n" << */
    /*       "Height: " << self_hector_map->info.height << " Width: " <<self_hector_map->info.width << "\n" << */
    /*       "Size of map data: " << self_hector_map->data.size()); */
    /*   pub_translated_map_.publish(*self_hector_map); */
    /* } */
    /* else */ 
    /*   ROS_ERROR("[TestController]: Did not receive the self hector map"); */

    // Laser scan
    /* sensor_msgs::LaserScanConstPtr self_laser_scan = common_data->getSelfLaserScan(); */
    /* if(self_laser_scan) { */
    /*   ROS_DEBUG_STREAM("[TestController]: Received self laser scan.\n" << */
    /*       "Range Min: " << self_laser_scan->range_min << " Max: " << self_laser_scan->range_max << "\n" << */
    /*       "Size of laser data: " << self_laser_scan->ranges.size()); */
    /* } */
    /* else */ 
    /*   ROS_ERROR("[TestController]: Did not receive the self laser scan"); */

    //  Neighbor States
    swm_utils::IdStateArrayStampedConstPtr neighbor_states = common_data->getNeighborStates();
    if(neighbor_states) {
      ROS_DEBUG_STREAM("[TestController]: Received " << neighbor_states->states.size() << " neighbor states.");
      for(auto it = neighbor_states->states.begin(); it != neighbor_states->states.end(); it++) {
        ROS_DEBUG_STREAM("[TestController]: Neighbor ID: " << it->id << "\n" <<
            "Pose: (" << it->pose.position.x <<", "<<it->pose.position.y <<", "<<it->pose.position.z <<")");
      }
    }
    else { 
      ROS_ERROR("[TestController]: Did not receive any neighbor states");
    }

    /* path mode test //{ */

    if (_test_mode_ == "path") {

      mrs_msgs::Path path_cmd;

      path_cmd.header = swm_r_utils::createHeader(self_state->header.frame_id, ros::Time::now() + ros::Duration(_traj_gen_time_));

      if (!test_init_) {

        test_init_ = true;
        e::Vector3d init_pose(_polygon_side_, _polygon_side_, _polygon_side_);
        desired_points_ = getTriangle(init_pose, _polygon_side_);

        for (const auto &pt : desired_points_) {
          path_cmd.points.emplace_back(swm_r_utils::createReference(swm_r_utils::pointFromEigen(pt), 0.0));
        }
        cur_point_idx_ = 1;
      }
      else {
        e::Vector3d cur_pose = swm_r_utils::pointToEigen(self_state->state.pose.position);

        if ((cur_pose - desired_points_[cur_point_idx_]).norm() <= 0.1) {
          cur_point_idx_ = (cur_point_idx_ + 1) % desired_points_.size();
        }

        for (int i = 0; i < desired_points_.size(); ++i) {
          path_cmd.points.emplace_back(swm_r_utils::createReference(swm_r_utils::pointFromEigen(desired_points_[(cur_point_idx_ + i) % desired_points_.size()]), 0.0));
        }
      }

      path_cmd.input_id = 1;
      path_cmd.use_heading = false;
      path_cmd.fly_now = true;
      path_cmd.stop_at_waypoints = false;
      path_cmd.loop = false;
      path_cmd.override_constraints = false;
      path_cmd.relax_heading = false;

      ROS_DEBUG_STREAM("[TestController]: Desired Points: \n" <<
          "(" << desired_points_[0](0) <<", " << desired_points_[0](1) << ", " << desired_points_[0](2) << ")\n" <<
          "(" << desired_points_[1](0) <<", " << desired_points_[1](1) << ", " << desired_points_[1](2) << ")\n" <<
          "(" << desired_points_[2](0) <<", " << desired_points_[2](1) << ", " << desired_points_[2](2) << ")");

      return std::any{path_cmd};
    }

    //}

    /* velocity mode test //{ */

    if (_test_mode_ == "velocity") {

      mrs_msgs::VelocityReferenceStamped vel_cmd;
      vel_cmd.header = swm_r_utils::createHeader(self_state->header.frame_id, ros::Time::now());

      if (!test_init_) {
        test_init_ = true;
        e::Vector3d init_pose = swm_r_utils::pointToEigen(self_state->state.pose.position); 
        circle_center_ = e::Vector2d(init_pose(0) - _polygon_side_, init_pose(1));

        ROS_DEBUG_STREAM("[TestController]: Circle center: \n" <<
            "(" << circle_center_(0) <<", " << circle_center_(1) << ")");
      }

      e::Vector2d cur_pose(self_state->state.pose.position.x, self_state->state.pose.position.y); 
      e::Vector2d radial_vec = cur_pose - circle_center_;
      e::Rotation2Dd rot_90(M_PI/2.0);
      e::Vector2d tan_vel_vec = _polygon_side_ * (rot_90 * radial_vec.normalized());
      e::Vector2d rad_vel_vec = _polygon_side_ * radial_vec.normalized() - radial_vec;
      e::Vector2d net_vel = tan_vel_vec + rad_vel_vec;

      ROS_DEBUG_STREAM("[TestController]: Radial vector: \n" <<
          "(" << radial_vec(0) <<", " << radial_vec(1) << ")");

      vel_cmd.reference.velocity = swm_r_utils::vector3FromEigen(e::Vector3d(net_vel(0), net_vel(1), 0.0));
      vel_cmd.reference.use_heading = false;
      vel_cmd.reference.use_heading_rate = true;
      vel_cmd.reference.heading_rate = 0.5;

      return std::any{vel_cmd};
    }

    //}

    return std::nullopt;

  }

  //}

}

PLUGINLIB_EXPORT_CLASS(test_controller::TestController, swarm_control_manager::SwarmController)
