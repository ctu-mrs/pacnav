# MRS-SWARM-CORE

## What?
The mrs_swarm_core is a collection of pkgs (ROS1 and otherwise). It simplifies the interaction between ros nodes in the mrs_uav_system and presents a higher level interface to develop algorithms for multi-robot systems. The pkgs are designed to be ready to deploy with the mrs_uav_system without any changes to the codebase. This has drastically reduced the time from idea to implementation.

## Key ideas and features
- assumes the algorithm to be implemented as a ros nodelet 
- the nodelet receives odometry and other essential information about other robots as input and is expected to return an appropriate control command as output
- mrs_swarm_core also provides several ready to use pkgs for arbitrary tf connections, scheduling large scale, repeated experiments and obstacle avoidance etc.

## Package Descriptions
### swarm_control_manager
- provides a nodelet manager interface for dynamically loading and unloading custom algorithms 
- collects all the information from different nodes and provides them as a unified input to the custom algorithm
- uses the control command from the custom algorithm to control the UAV 
- performs several checks to avoid common mistakes during real-world experiments 

### swarm_utils 
- provides common_tf_pub for attaching several tfs to a new frame, thus creating a global/common reference frame
- provides math_utils for commonly used math operations, for eg., sampling random vectors from a distribution
- provides ros_utils for eary generation of rosmsgs and conversions to other data structures
- provides shared_gps_aggr to aggregate GPS position msgs shared over a common network
- provides scheduler to schedule automated repeated experiments for a custom algorithm

### swarm_gazebo_resources
- provides commonly used gazebo models 
- provides large gazebo models scanned from different real-world locations (download_large_model.sh)

### forest_path_finder
- provides an Jump Point Search based grid path planner for 2D path planning

## Instructions to use mrs_swarm_core
---
### Installation 
Run the script `installation/install.sh` to install all the required packages and gazebo resources (e.g worlds, models etc.).

### Simulating a swarm using your controller 
- use the config file in `simulation/config/sim_config.yaml` to define the parameters for your experiment
- run the simulation using `./simulate_swarm.sh -f config/sim_config.yaml` in `simulation` directory
