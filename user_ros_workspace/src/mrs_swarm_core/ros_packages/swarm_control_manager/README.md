# SWARM CONTROL MANAGER

## Overview (the why?)

- provides a simple interface to use the [mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system) without worrying about different underlying subsystems
- plugin based design makes it possible to use multiple control methods during the same flight
- makes the development, debugging and testing of algorithms easier by separating the swarm algorithm from [mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system)

## Useful properties

- possible to switch, activate and deactivate different control methods online
- extra safety checks for beginners with [mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system)

## Available control modes

Currently two control modes are supported by the **swarm control manager**.
They provide reasonable control of the UAV agent to test most swarm algorithms in simulation and real world experiments.
But the [mrs_uav_system](https://github.com/ctu-mrs/mrs_uav_system) provides higher level of control.
All the available control modes provide an option to rotate the UAV agent around its Z-axis to search for other agents. This is only useful when using cameras with F.O.V constraints e.g. UVDAR.

- **Position Mode**
  - assumes that the swarm algorithm only depends on the current position of the agent
  - **pros**: only needs desired position and heading of the agent
  - **cons**: easy and safe of the UAV agent

- **Velocity Mode**
  - assumes that the swarm algorithm depends on position and Velocity of the agent i.e models the agent as a **Single Integrator**
  - **pros**: needs desired Velocity and heading of the agent
  - **cons**: shifts more responsibility to the swarm algorithm

## Getting started

A test controller implemented as a standalone plugin is available as a reference to start building your own swarm control algorithm.
The **swarm control manager** uses a config file ``config/swarm_controller.yaml`` to identify, register and load the required controller.



