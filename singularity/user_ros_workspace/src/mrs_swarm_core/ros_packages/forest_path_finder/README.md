# Path Finder

## Description
The package builds a path for the drone from a grid map of obstacles supplied to it. 

## Implementation details

### Grid 
**Grid** is a class used for storing grid of obstacles and performing actions with it. User specifies the width, height and resolution for the grid to create one.  

It uses class `Coordinate(x, y)` for coordinates inside the grid.  

**Funcions**:  
* `operator()(y, x)` or operator()(Coordinate) - for getting and setting the data in grid
* map() and forEach()
* translateToCoordinates() and translateFromCoordinates() to switch between coordinates on the grid and real coordinates of the drone.
* createDistanceGrid() - function returns a new Grid of floats with the distnace to nearest obstacle as value in each cell.
* createVoronoiGraph() - function returns a new Grid, that has a voronoi map for Grid. The grid has `1` in points that are the furthest points between 2 closest obstacles (the definition of voronoi but applied to grid) it also makes all cells that are closer to obstalces than `minSafeDistance` invalid and all further than `maxCautionDistance` valid.

### UvdarListener
**UvdarListener** is a class that listens to uvdar and returns the last valid positions of other drones to user when asked. The constructor uses `ros::NodeHandle` of your node.   

The following parameters should be set for this node:  
* `uvdar_required_frame_id` - the frame to which to translate the coordinates.
* `uvdar_invalidate_time` - if we don't see the drone for longer than this time - its position becomes invalid and is not returned (defaults to 0.5).
* `uvdar_subscriber_ids` - the topics to which to listen for other drones positions (this is an array of strings) (the uvdar publishes to `/[uav_name]/uvdar/filteredPose[number]`.

You can get the other drones positions using `getStates()` and `getValidStates()` functions. The `getValidStates` only returns the states that were received recently.  

### RosMessageConvertors
**RosMessageConvertors** is just a small library for creating ros messages from other cpp structures used to simplified the code.

### PathFindindingGeneral
A library that has path finding algorithm for grid implemented inside. It finds the path using **jumpPointSearch**, which is a modified version of **aStar** that works better on grid. The main function is `findPath()`.  

### Spline
A library that creates cubic spline between given points. It is used to make the path smoother from the created path on grid.  

### PathFinder
The main node that receives the `hector_map` and drone position from `position_cmd`. It then publishes the result as a message to `/uav/path_finder/path`. It receives the command where to go to `/uav/path_finder/path_goto`. Inside it creates a voronoi diagram, builds the path and makes it smoother using cubic spline.  
The node can be used to directly control the drone if you uncomment the corresponding define in `forest_path_finder.cpp`: `#define FLIGHT` and rebuild.

