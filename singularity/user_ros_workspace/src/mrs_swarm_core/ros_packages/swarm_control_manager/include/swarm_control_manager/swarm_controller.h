#pragma once
#ifndef SWARM_CONTROLLER_H
#define SWARM_CONTROLLER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <any>

#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>

#include <swarm_control_manager/common_data_handler.h>
#include <swarm_utils/ControlCommand.h>

namespace swm_ctrl = swarm_control_manager;
namespace utils = swarm_utils;

namespace swarm_control_manager
{

  // Class to be inherited by all the swarm controllers
  // DO NOT REMOVE THE = 0 for the functions
  class SwarmController{
    public:
      virtual void initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& ros_name_space, std::shared_ptr<mrs_lib::Transformer> transformer) = 0;
      virtual void activate() = 0;
      virtual void deactivate() = 0;
      virtual std::optional<std::any> update(std::shared_ptr<swm_ctrl::SwarmCommonDataHandler> common_data) = 0;

      virtual ~SwarmController() = 0;
  };

// A pure virtual destructor requires a function body.
  SwarmController::~SwarmController() {};
}

#endif
