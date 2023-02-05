# MRS-SWARM-CORE
This is a collection of packages and resources to design and experiment with various multi-robot and swarming algorithms. It serves as an interface between the mrs_uav_system and your custom algorithm. It also provides some utility nodes that may prove useful for your custom algorithm. The swarm_core contains the following packages:

## Package Descriptions
#### swarm_control_manager
-

# ## Instructions to use mrs_swarm_core
---
### Installation 
Run the script `installation/install.sh` to install all the required packages and gazebo resources (e.g worlds, models etc.).

### Package overview
* `forest_path_finder`
  - Path finder implementing A-star and Jump-Point search for 2D grid. It is used as the path planner for both boids and colass package.
* `multi_uav_dynamic_reconfig`
  - It is used to reconfigure parameters for multiple UAVs simultaneously using a single interface.
* `swarm_control_manager`
  - It is the main package that handles the control of the swarm and is desinged to streamline and simplify the process of integrating and testing different swarm control approaches. This also makes it easier to deploy these algorithms to real world without any hustle. It acts as a middleware between the `mrs_uav_system` and your swarm controller.
* `swarm_utils`
  - It contains utility nodes that are used for simulation and real world deployment. It also has some utility functions for creating common msg types and their conversions.
* `swarm_gazebo_resources`
  - It contains all the models and world files generally used by swarm experiments.

### Simulating a swarm using your controller 
*to be continued*
