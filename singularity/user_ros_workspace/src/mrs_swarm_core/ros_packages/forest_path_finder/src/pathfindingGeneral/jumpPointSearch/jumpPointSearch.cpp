/* includes //{ */

#include "jumpPointSearch.h"

#include <stdint.h>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <tuple>
#include <iostream>

//}

/* helper //{ */

const float sqrt2 = std::sqrt(2);

const float MAX_DISTANCE = __UINT32_MAX__;

template <typename T>
constexpr int8_t numberSign(T value) {
  return value > 0 ? 1 : value < 0 ? -1 : 0;
}

//}

/* GridSafeWrapper //{ */

template <typename T>
class GridSafeWrapper {
 const Grid<T>& grid;

public:
  GridSafeWrapper(const Grid<T>& grid): grid(grid) {}

  const T operator()(const uint32_t y, const uint32_t x) const {
    if (y < 0 || x < 0 || y >= grid.getHeight() || x >= grid.getWidth())
      return false;
    return grid(y, x);
  }

  const T operator()(Coordinate& coordinate) const {
    if (coordinate.y < 0 || coordinate.x < 0 || 
        coordinate.y >= grid.getHeight() || coordinate.x >= grid.getWidth())
      return false;
    return grid(coordinate);
  }
};

//}

/* JumpPoint //{ */

struct JumpPoint {
  Coordinate coordinate;
  float startDistance;
  float totalDistanceEst;
  size_t parent;

  JumpPoint() = default;
  JumpPoint(
      Coordinate coordinate, 
      float startDistance = MAX_DISTANCE, 
      float totalDistanceEst = MAX_DISTANCE, 
      size_t parent = 0):
    coordinate(coordinate), startDistance(startDistance), totalDistanceEst(totalDistanceEst), parent(parent) {};
};

//}

/* Comparator //{ */

class Comparator {
  const std::vector<JumpPoint>& jumpPoints;

public:
  Comparator(const std::vector<JumpPoint>& jumpPoints): jumpPoints(jumpPoints) {}

  // should a be later in queue than b
  bool operator()(const size_t a, const size_t b) const {
    return jumpPoints[a].totalDistanceEst > jumpPoints[b].totalDistanceEst;
  }
};

//}

/* shortestDistance() //{ */

float shortestDistance(const Coordinate& start, const Coordinate& finish) 
{
  // Finding the shortest distance possible on grid - only straight or diagonal moves
  // Could be finding euclidean distance instead
  // Not sure which one is faster
  float vertical_dist = std::abs((int64_t) finish.y - start.y);
  float horizontal_dist = std::abs((int64_t) finish.x - start.x);
  float difference = std::abs(horizontal_dist - vertical_dist);
  float common = (horizontal_dist + vertical_dist - difference) / 2;

  return difference + common * sqrt2;
}

//}

/* getNeighbors() //{ */

std::vector<Coordinate> getNeighbors(JumpPoint& point, const std::vector<JumpPoint>& jumpPoints, const GridSafeWrapper<bool>& obstacleGrid) {
  std::vector<Coordinate> neighbors{};

  uint32_t y = point.coordinate.y;
  uint32_t x = point.coordinate.x;
  
  if (!point.parent) {
    for (int8_t dy = -1; dy <= 1; ++dy) {
      for (int8_t dx = -1; dx <= 1; ++dx) {
        if ((dy != 0 || dx != 0) && obstacleGrid(y + dy, x + dx)) 
          neighbors.emplace_back(x + dx, y + dy);
      }
    }
  } else {
    uint32_t parentY = jumpPoints[point.parent].coordinate.y;
    uint32_t parentX = jumpPoints[point.parent].coordinate.x;
    int8_t differenceY = numberSign((int64_t) y - parentY);
    int8_t differenceX = numberSign((int64_t) x - parentX);

    if (differenceX && differenceY) {
      // diagonal move
      // natural neighbors
      if (obstacleGrid(y + differenceY, x + differenceX))
        neighbors.emplace_back(x + differenceX, y + differenceY) ;
      if (obstacleGrid(y + differenceY, x))
        neighbors.emplace_back(x, y + differenceY);
      if (obstacleGrid(y, x + differenceX))
        neighbors.emplace_back(x + differenceX, y);

      // forced neighbors
      if (!obstacleGrid(y - differenceY, x) && 
          obstacleGrid(y - differenceY, x + differenceX))
        neighbors.emplace_back(x + differenceX, y - differenceY);
      if (!obstacleGrid(y, x - differenceX) &&
          obstacleGrid(y + differenceY, x - differenceX))
        neighbors.emplace_back(x - differenceX, y + differenceY);
    } else {
      // straight move
      // natural neighbors
      if (obstacleGrid(y + differenceY, x + differenceX)) 
          neighbors.emplace_back(x + differenceX, y + differenceY);

      // forced neighbors
      if (!obstacleGrid(y + differenceX, x + differenceY) && 
          obstacleGrid(y + differenceX + differenceY, x + differenceY + differenceX)) 
        neighbors.emplace_back(x + differenceY + differenceX, y + differenceX + differenceY);
 
      if (!obstacleGrid(y - differenceX, x - differenceY) && 
          obstacleGrid(y - differenceX + differenceY, x - differenceY + differenceX)) 
        neighbors.emplace_back(x - differenceY + differenceX, y - differenceX + differenceY) ; 
    }
  }

  return neighbors;
}

//}

/* jump() //{ */

std::tuple<Coordinate, bool> jump(
    Coordinate& parent, Coordinate& child, 
    const GridSafeWrapper<bool>& obstacleGrid, Coordinate& finish, 
    Coordinate& closest) 
{
  int8_t differenceY = numberSign((int64_t) child.y - parent.y);
  int8_t differenceX = numberSign((int64_t) child.x - parent.x);

  // invalid
  if (!obstacleGrid(child)) {
    // If the parent is closest to finish - make it a new point
    if (shortestDistance(parent, finish)) { 
      return std::make_tuple(parent, true);
    }
    // return false - no point here needed
    return std::make_tuple(Coordinate{}, false);
  }

  if (child == finish)
    return std::make_tuple(child, true);

  // there are forced neighbors
  bool forced = false;
  if (differenceX && differenceY) {
    // diagonal move
    if ((!obstacleGrid(child.y - differenceY, child.x) && 
          obstacleGrid(child.y - differenceY, child.x + differenceX)) ||
        (!obstacleGrid(child.y, child.x - differenceX) &&
          obstacleGrid(child.y + differenceY, child.x - differenceX)))
      forced = true;
  } else {
    // straight move
    if ((!obstacleGrid(child.y + differenceX, child.x + differenceY) &&
          obstacleGrid(child.y + differenceX + differenceY, child.x + differenceY + differenceX)) ||
        (!obstacleGrid(child.y - differenceX, child.x - differenceY) &&
          obstacleGrid(child.y - differenceX + differenceY, child.x - differenceY + differenceX)))
      forced = true;
  }

  if (forced)
    return std::make_tuple(child, true);

  // diagonal
  if (differenceX && differenceY) {
    Coordinate move1{child.x + differenceX, child.y};
    Coordinate move2{child.x, child.y + differenceY};
    if (std::get<1>(jump(child, move1, obstacleGrid, finish, closest)) ||
        std::get<1>(jump(child, move2, obstacleGrid, finish, closest)))
      return std::make_tuple(child, true);
  }
  
  // move further
  Coordinate move{child.x + differenceX, child.y + differenceY} ;
  return jump(child, move, obstacleGrid, finish, closest);
}

//}

std::vector<Coordinate> jumpPointSearch(const Grid<bool> &grid, const Coordinate start, const Coordinate finish) 
{
  /* initialize //{ */

  GridSafeWrapper<bool> obstacleGrid{grid};
  
  // in queue and unordered map, size_t - index of vertice
  std::vector<JumpPoint> jumpPoints{};
  std::hash<Coordinate> hash{grid.getWidth(), grid.getHeight()};
  std::unordered_map<Coordinate, size_t> openVertices{10, hash};
  std::unordered_map<Coordinate, bool> closedVertices{10, hash};
  std::priority_queue<size_t, std::vector<size_t>, Comparator> queue{Comparator{jumpPoints}};

  JumpPoint startPoint { start, 0 };
  JumpPoint finishPoint { finish };
  startPoint.totalDistanceEst = shortestDistance(startPoint.coordinate, finishPoint.coordinate);
  jumpPoints.push_back(JumpPoint{}); // 0 index means nothing
  jumpPoints.push_back(startPoint); // 1 index is start point
  closedVertices[startPoint.coordinate] = true;
  queue.push(1); // add startPoint to queue
 
  std::vector<Coordinate> path{};
  if (!obstacleGrid(startPoint.coordinate)) {
    return path;
  }

  // Find closest point - should be near obstacle and closest to finish
  JumpPoint closestPoint = startPoint;

  //}
  
  /* main loop //{ */

  bool pathToFinishFound = false;

  while (!queue.empty()) {
    // save it in the vector
    size_t pointIndex = queue.top();
    JumpPoint point = jumpPoints[pointIndex];

    queue.pop();
    closedVertices[point.coordinate] = true;

    if (point.coordinate == finishPoint.coordinate) {
      finishPoint = point;
      pathToFinishFound = true;
      break;
    }
    // set the closest point if it is closest
    if (point.totalDistanceEst - point.startDistance < closestPoint.totalDistanceEst - closestPoint.startDistance) {
      closestPoint = point;
    }

    auto neighbors = getNeighbors(point, jumpPoints, obstacleGrid);
    for (Coordinate& neighbor: neighbors) {
      Coordinate jumpCoord;
      bool jumpSuccess;

      std::tie(jumpCoord, jumpSuccess) = jump(point.coordinate, neighbor, obstacleGrid, 
          finishPoint.coordinate, closestPoint.coordinate);
      if (!jumpSuccess || closedVertices[jumpCoord])
        continue;

      double startDistance = point.startDistance + shortestDistance(point.coordinate, jumpCoord);
      double totalDistanceEst = 
        startDistance + shortestDistance(jumpCoord, finishPoint.coordinate);
      
      size_t neighborPointIndex = openVertices[jumpCoord];
      if (!neighborPointIndex) {
        jumpPoints.emplace_back(jumpCoord, startDistance, totalDistanceEst, pointIndex);
        openVertices[jumpCoord] = jumpPoints.size() - 1;
        queue.push(jumpPoints.size() - 1);
      } else if (jumpPoints[neighborPointIndex].startDistance > startDistance) {
        jumpPoints[neighborPointIndex].startDistance = startDistance;
        jumpPoints[neighborPointIndex].totalDistanceEst = totalDistanceEst;
      }  
    }
  }

  //}
  
  /* retrieve path //{ */

  if (!pathToFinishFound)
    finishPoint = closestPoint;

  Coordinate previousCoordinate = finishPoint.coordinate;
  JumpPoint& point = finishPoint;
  std::vector<Coordinate> pathBackwards{};
  pathBackwards.push_back(point.coordinate);

  while (true) {
    // add all points including current point from previous coordinate
    while (previousCoordinate != point.coordinate) {
      previousCoordinate.x += numberSign((int64_t) point.coordinate.x - previousCoordinate.x);
      previousCoordinate.y += numberSign((int64_t) point.coordinate.y - previousCoordinate.y);
      
      pathBackwards.push_back(previousCoordinate);
    }

    if (point.coordinate == startPoint.coordinate) break;
    point = jumpPoints[point.parent];
  }

  // reverse it back
  path.insert(path.begin(), pathBackwards.rbegin(), pathBackwards.rend());

  //}
  
  return path;
}

