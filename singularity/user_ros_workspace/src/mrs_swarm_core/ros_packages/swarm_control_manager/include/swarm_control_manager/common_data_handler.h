#pragma once
#ifndef COMMONDATAHANDLER_H
#define COMMONDATAHANDLER_H

#include <mutex>

#include <mrs_msgs/PositionCommand.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>

#include <swarm_utils/IdStateStamped.h>
#include <swarm_utils/IdStateArrayStamped.h>

namespace utils = swarm_utils;

namespace swarm_control_manager
{

/* struct SwarmCommonDataHandler //{ */

struct SwarmCommonDataHandler {
  // Functions
  std::function<utils::IdStateStampedConstPtr()> getSelfState;
  std::function<nav_msgs::OccupancyGridConstPtr()> getSelfHectorMap;
  std::function<sensor_msgs::LaserScanConstPtr()> getSelfLaserScan;
  std::function<utils::IdStateArrayStampedConstPtr()> getNeighborStates;
};

//}

}

#endif
