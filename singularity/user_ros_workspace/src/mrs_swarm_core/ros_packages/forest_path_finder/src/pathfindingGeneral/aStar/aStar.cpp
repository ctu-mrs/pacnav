/* includes //{ */

#include "aStar.h"

#include <cmath>
#include <queue>
#include <tuple>
#include <stdint.h>

//}

const float sqrt2 = std::sqrt(2);
const float MAX_DISTANCE = __UINT32_MAX__;

using queueType = std::tuple<Coordinate, float>;

/* GridCell struct //{ */

struct GridCell
{
    float startDistance;
    float distanceEstimate;

    GridCell() : startDistance(MAX_DISTANCE), distanceEstimate(MAX_DISTANCE) {
    }

    GridCell(float startDistance, float distanceEstimate) : startDistance(startDistance), distanceEstimate(distanceEstimate) {
    }

    friend std::vector<Coordinate> aStar(Grid<bool>& grid, float, GridCell, GridCell);
};

//}

/* getNeighbours() //{ */

// get neighbors for a grid coordinate
template <typename T>
std::vector<Coordinate> getNeighbors(const Coordinate& coord, const Grid<T>& grid) {
  std::vector<Coordinate> neighbors;
  neighbors.reserve(8);

  for (int8_t dy = -1; dy <= 1; ++dy) {
    for (int8_t dx = -1; dx <= 1; ++dx) {
      if ((dx == 0 && dy == 0) ||
          coord.x + dx < 0 || coord.x + dx >= grid.getWidth() ||
          coord.y + dy < 0 || coord.y + dy >= grid.getHeight()) {
        continue;
      }

      neighbors.emplace_back(coord.x + dx, coord.y + dy);
    }
  }

  return neighbors;
}

//}

/* shortestDistance() //{ */

float shortestDistance(const Coordinate& start, const Coordinate& finish) {
  float vertical_dist = std::abs((int64_t)finish.x - start.x);
  float horizontal_dist = std::abs((int64_t)finish.y - start.y);
  float difference = std::abs(horizontal_dist - vertical_dist);
  float common = (horizontal_dist + vertical_dist - difference) / 2;

  return difference + common * sqrt2;
}

//}

/* Comparator class //{ */

class Comparator {
public:
    bool operator()(const queueType& a, const queueType& b) {
      return std::get<1>(a) > std::get<1>(b);
    };
};

//}

std::vector<Coordinate> aStar (const Grid<bool>& grid, const Coordinate start, const Coordinate finish)
{
  /* initialization //{ */

  // initialize a container that will contain calculated distances
  Grid<GridCell> vertices{grid.getWidth(), grid.getHeight(), grid.getResolution()};

  // a priority queue of vertices that need to be visited
  // auto comparator = [&vertices](Coordinate& a, Coordinate& b) { return vertices(a).distanceEstimate > vertices(b).distanceEstimate; };
  auto comparator = Comparator();
  std::priority_queue<queueType, std::vector<queueType>, decltype(comparator)> openVertices{comparator};
  Grid<bool> isInOpenVertices{grid.getWidth(), grid.getHeight(), grid.getResolution()};

  // a bitmask of the vertices that were already used
  Grid<bool> closedVerticesMask{grid.getWidth(), grid.getHeight(), grid.getResolution()};
  for (uint32_t y = 0; y < vertices.getWidth(); ++y) {
    for (uint32_t x = 0; x < vertices.getHeight(); ++x) {
      if (!grid(y, x))
        closedVerticesMask(y, x) = 1;
    }
  }

  if (closedVerticesMask(start) || closedVerticesMask(finish))
    return std::vector<Coordinate>{};
  
  // put the distances of start vertice
  vertices(start).startDistance    = 0;
  vertices(start).distanceEstimate = shortestDistance(start, finish);
  openVertices.push(std::make_tuple(start, vertices(start).distanceEstimate));

  //}

  /* main loop //{ */

  bool pathToFinishFound = false;

  // This will be used if exact path to finish is not found
  while (!openVertices.empty()) {
    Coordinate top = std::get<0>(openVertices.top());
    openVertices.pop();
    closedVerticesMask(top) = 1;

    if (top == finish) {
      pathToFinishFound = true;
      break;
    }

    auto neighbors = getNeighbors(top, grid);
    for (Coordinate& neighbor : neighbors) {
      // skip if already calculated
      if (closedVerticesMask(neighbor))
        continue;

      // recalculate the distances
      GridCell& neighborCell  = vertices(neighbor);
      float extraDistance = neighbor.x == top.x || neighbor.y == top.y ? 1 : sqrt2;
      float startDistance = vertices(top).startDistance + extraDistance;
      if (startDistance < neighborCell.startDistance) {
        neighborCell.startDistance    = startDistance;
        neighborCell.distanceEstimate = startDistance + shortestDistance(neighbor, finish);
      }

      // add to open vertices
      if (!isInOpenVertices(neighbor)) {
        isInOpenVertices(neighbor) = 1;
        openVertices.push(std::make_tuple(std::move(neighbor), neighborCell.distanceEstimate));
      }
    }
  }

  //}

  /* retrieve path //{ */

  std::vector<Coordinate> path;

  if (!pathToFinishFound)
    return path;

  // rebuild path
  std::vector<Coordinate> backPath{};
  Coordinate currentVertice = finish;
  while (true) {
    backPath.push_back(currentVertice);
    if (currentVertice == start)
      break;

    auto neighbors = getNeighbors(currentVertice, grid);
    float minDistance = MAX_DISTANCE;
    for (auto neighbor : neighbors) {
      if (vertices(neighbor).startDistance < minDistance) {
        currentVertice = neighbor;
        minDistance = vertices(neighbor).startDistance;
      }
    }
  }

  // reverse the path
  path.insert(path.end(), backPath.rbegin(), backPath.rend());
  return path;

  //}
}
