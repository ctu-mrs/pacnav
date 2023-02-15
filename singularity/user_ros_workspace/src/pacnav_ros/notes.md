# Notes about the ideas behind the method
- This method tries to solve the problem of cooperative navigation in a group of agents 
  assuming single integrator dynamics.
- One or more agents are informed about a single goal position while other agents remain
  uninformed.
- The task is to safely navigate the entrie group of agents to the provided goal.
- Baisc idea is to select a good target among the known neighbors and follow it.
- The target can also be static goal, provided by a human or a moving goal to track.
- The target selection reduces the problem to leader following, once a target is selected.
- The cooperative navigation is encapsulated in the target selection process.
  It takes inspiration from the cooperative motion in herds of animals.
- It can be seen as an extension of simple boids models, where only current position and 
  velocity are used for cooperation. The assumption is that motion history provided 
  richer information for cooperative navigation than just velocity.

## Components of the problem

### Target selection
- The agent records the path history of surrounding neighbors and finds similarity b/w them.
- Each neighbor has a similarity score (coorelation) which denotes the extent of similarity 
  b/w the motion history of the neighbor with rest of the group.
- ~~Each neighbor also has a persistence score which denotes the extent of consistency in the motion history of the neighbor.~~
- The neighbor with highest similarity score is selected as the target.

### Calculating the control command
- After the target selction, either a path planning method or simple relative vector is used
  to calculate the velocity command for the agent.
- The desired velocity is the vector sum of obstalce avoidance vectors and the navigation 
  vector from planned path.
- Inter-agent obstalce avoidance is solved by treating the neighbor agents as dynamic obstacles.
- A preventive (rescaling) method is used to prevent the informed agent to leave the group behind.

## Implementation notes
- Any data that is stored for later use or is maintained as a private variable, must be stored in
  **origin_frame** because the data in **uav_frame** will move with the UAV. The **origin_frame**
  is also significantly more stable.
  
## Current challenges and issues
- unlike the boids model, the state of the swarm with no goal is not static but the agents move
  as the target inference changes more randomly
- the blind spot of UVDAR still creates deadlocks but they are slowly overcome by tangential avoidance vectors
- the inference jumps a lot when the neighbor path estimate is bad. Jumping target results in oscillatory motion which slows down the agent.
