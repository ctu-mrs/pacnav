#ifndef FOREST_PATH_FINDER_PATHFINDINGGENERAL_H
#define FOREST_PATH_FINDER_PATHFINDINGGENERAL_H

/* includes //{ */

#include <iostream>
#include <queue>

#include "path_finder/grid.h"
#include "aStar/aStar.h"
#include "jumpPointSearch/jumpPointSearch.h"
/* #include "jps.h" */

//}

/* getClosestValidCell() //{ */

static const float sqrt2 = std::sqrt(2);

/* getNeighbors() //{ */

static std::vector<Coordinate> getNeighbors(Coordinate& coordinate, const Grid<bool>& grid) {
  uint32_t x = coordinate.x, y = coordinate.y;
  std::vector<Coordinate> result;

  for (int8_t dy = -1; dy <= 1; ++dy) {
    for (int8_t dx = -1; dx <= 1; ++dx) {
      if ((dx != 0 || dy != 0) && 
          x + dx >= 0 && x + dx < grid.getWidth() && y + dy >= 0 && y + dy < grid.getHeight())
        result.emplace_back(x + dx, y + dy);
    }
  }

  return result;
}

//}

static std::tuple<bool, std::vector<Coordinate>, Coordinate> getClosestValidCell
        (const Grid<bool>& obstacleGrid, Coordinate coordinate)
{
  // function finds the closest valid vertice to given
  // returns a boolean weather it is possible to find one and the path to it
  Grid<bool> openVertices{obstacleGrid.getWidth(), obstacleGrid.getHeight(), obstacleGrid.getResolution()};
  std::vector<Coordinate> path;
  
  if (obstacleGrid(coordinate)){
    return std::make_tuple(true, path, coordinate);
  }

  using cell = std::tuple<Coordinate, float>;
  auto comparator = [](cell a, cell b) { return std::get<1>(a) > std::get<1>(b); };
  std::priority_queue<cell, std::vector<cell>, decltype(comparator)> searchVertices { comparator };
  searchVertices.push(std::make_tuple(coordinate, 0));

  while(!searchVertices.empty()) {
    cell top = searchVertices.top();
    searchVertices.pop();
    Coordinate topCoord = std::get<0>(top);
    float topDistance = std::get<1>(top);

    // found path - return result
    if (obstacleGrid(topCoord)) {
      std::vector<Coordinate> result;

      while (topCoord != coordinate) {
        result.push_back(coordinate);
        coordinate.x += sign((int64_t) topCoord.x - coordinate.x);
        coordinate.y += sign((int64_t) topCoord.y - coordinate.y);
      }

      return make_tuple(true, result, topCoord);
    }

    auto neighbors = getNeighbors(topCoord, obstacleGrid);

    for (auto neighbor: neighbors) {
      if (openVertices(neighbor)) continue;

      const float extraDistance = neighbor.x == topCoord.x || neighbor.y == topCoord.y ? 1 : sqrt2;
      openVertices(neighbor) = true;
      searchVertices.push(std::make_tuple(neighbor, topDistance + extraDistance));
    }
  }

  // didn't find path - return nothing
  return std::make_tuple(false, path, Coordinate{0, 0});
}

//}

/* libraryJumpPointSearch() //{ */

/* GridWrapper for library //{ */
/*

class GridWrapper {
    const Grid<bool>&    grid;

public:
    GridWrapper(const Grid<bool>& grid): grid(grid) {}

    bool operator()(unsigned x, unsigned y) const {
      if (x < grid.getWidth() && y < grid.getHeight()) {
        return grid(y, x);
      }
      return false;
    }
};

*/
//}
/*

std::vector<Coordinate> libraryJumpPointSearch(const Grid<bool>& grid, const Coordinate start, const Coordinate finish) {
  GridWrapper gridWrapper{grid};
  JPS::PathVector path;
  bool found = JPS::findPath(path, gridWrapper, start.x, start.y, finish.x, finish.y, 1);

  std::vector<Coordinate> result;
  if (!found) return result;
  for (auto& point: path) {
    result.emplace_back(point.x, point.y);
  }

  return result;
}

*/
//}

/* PathFindingResult //{ */

struct PathFindingResult {
    std::vector<Coordinate> path;
    bool startModified;
    bool finishModified;

    PathFindingResult() = default;
    PathFindingResult(const std::vector<Coordinate> &path, bool startModified, bool finishModified)
            : path(path), startModified(startModified),finishModified(finishModified) {}
};

//}

// change this to change algorithm
// options: aStar(), jumpPointSearch(), libraryJumpPointSearch()
// NOTE: jumpPointSearch will return path to closest point if exact could not be found

typedef std::vector<Coordinate> (* algorithm_t)(const Grid<bool>&, const Coordinate, const Coordinate);
algorithm_t pathFindingAlgorithm = jumpPointSearch;

/* findPath() //{ */

// obstacleGrid - true if can walk on coordinate
PathFindingResult findPath(const Grid<bool>& obstacleGrid, Coordinate start, Coordinate finish) {
  std::vector<Coordinate> result;

  bool startFound; 
  std::vector<Coordinate> pathToStart;
  Coordinate validStart;
  std::tie(startFound, pathToStart, validStart) = getClosestValidCell(obstacleGrid, start);

  /* bool finishFound; */
  /* std::vector<Coordinate> pathToFinish; */
  /* Coordinate validFinish; */
  /* std::tie(finishFound, pathToFinish, validFinish) = getClosestValidCell(obstacleGrid, finish); */

  if (!startFound) {
    return PathFindingResult{result, false, false};
  }

  std::vector<Coordinate> path = 
    pathFindingAlgorithm(obstacleGrid, validStart, finish);

  if (path.empty()) {
    return PathFindingResult{result, false, false};
  }
  bool finishModified = finish != path[path.size() - 1];

  result.insert(result.end(), pathToStart.begin(), pathToStart.end());
  result.insert(result.end(), path.begin(), path.end());
  
  // if pathToStart is empty - the start coordinate was valid
  return PathFindingResult{result, !pathToStart.empty(), finishModified};
}

//}

#endif //FOREST_PATH_FINDER_PATHFINDINGGENERAL_H
