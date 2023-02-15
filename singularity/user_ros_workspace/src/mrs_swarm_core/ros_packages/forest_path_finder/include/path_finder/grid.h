#ifndef FOREST_PATH_FINDER_GRID_H
#define FOREST_PATH_FINDER_GRID_H

/* includes //{ */

#include <stdint.h>
#include <cmath>
#include <vector>
#include <functional>
#include <eigen3/Eigen/Eigen>

#include <iostream>

//}

/* helper functions //{ */

static inline constexpr float length2(float x, float y) {
  return (x * x) + (y * y);
}

template<typename T>
static inline constexpr int8_t sign(T x) {
  return x > 0 ? 1 : x < 0 ? -1 : 0;
}

template<typename T>
static inline constexpr T sqr(T x) {
  return x * x;
}

//}

/* Coordinate struct //{ */

struct Coordinate
{
    uint32_t x;
    uint32_t y;
    Coordinate(uint32_t x, uint32_t y) : x(x), y(y) {}
    Coordinate(): x(0), y(0) {}

    friend bool operator==(const Coordinate& first, const Coordinate& second) {
      return first.x == second.x && first.y == second.y;
    }
    friend bool operator!=(const Coordinate& first, const Coordinate& second) {
      return first.x != second.x || first.y != second.y;
    }

    friend std::ostream& operator<<(std::ostream& stream, const Coordinate& coord) {
      stream << "[x: " << coord.x << ", y: " << coord.y << "]";
      return stream;
    }
};

/* hash specification //{ */

namespace std {
  template<>
  struct hash<Coordinate> {
    uint32_t width, height; 

    hash(uint32_t width, uint32_t height): 
      width(width), height(height) {}; 

    size_t operator()(const Coordinate& coordinate) const {
      return coordinate.y * width + coordinate.x;
    }
  };
}

//}

//}

template <typename T>
class Grid {
private:
    uint32_t       width;
    uint32_t       height;
    float          resolution;
    std::vector<T> data;

public:
    /* constructor() //{ */

    Grid(uint32_t width, uint32_t height, float resolution, std::vector<T> data) : width(width), height(height), resolution(resolution), data(data) {
    }

    Grid(uint32_t width, uint32_t height, float resolution) : width(width), height(height), resolution(resolution), data(width * height) {
    }

    Grid(uint32_t width, uint32_t height, float resolution, T value) : width(width), height(height), resolution(resolution) {
      data.insert(data.begin(), width * height, value);
    }

    template <typename P>
    explicit Grid(const Grid<P>& other) : width(other.getWidth()), height(other.getHeight()), resolution(other.getResolution()), data(other.getData().size()) {
      for (size_t i = 0; i < other.data.size(); ++i)
        data[i] = static_cast<T>(other.data[i]);
    }

    //}

    /* getters and setters //{ */

    std::vector<T>& getData() {
      return data;
    }
    uint32_t getWidth() const {
      return width;
    }
    uint32_t getHeight() const {
      return height;
    }
    float getResolution() const {
      return resolution;
    }

    decltype(data[0]) operator()(uint32_t y, uint32_t x) {
      return data[y * width + x];
    }

    const T operator()(uint32_t y, uint32_t x) const {
      return data[y * width + x];
    }

    decltype(data[0]) operator()(const Coordinate& coord) {
      return data[coord.y * width + coord.x];
    }
 
    const T operator()(const Coordinate& coord) const {
      return data[coord.y * width + coord.x];
    }

    //}

    /* map() & forEach() //{ */

    void forEach(std::function<T(T)> function) {
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          this->operator()(y, x) = function(this->operator()(y, x));
        }
      }
    }

    void forEach(std::function<T(T, uint32_t, uint32_t)> function) {
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          this->operator()(y, x) = function(this->operator()(y, x), x, y);
        }
      }
    }

    template<typename S>
    Grid<S> map(std::function<S(T)> function) const {
      Grid<S> result{width, height, resolution};
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          result(y, x) = function(this->operator()(y, x));
        }
      }
      return result;
    }

    template<typename S>
    Grid<S> map(std::function<S(T, uint32_t x, uint32_t y)> function) const {
      Grid<S> result{width, height, resolution};
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          result(y, x) = function(this->operator()(y, x), x, y);
        }
      }
      return result;
    }

    //}

    /* translateToCoordinates(Vector3) -> Coordinate //{ */

    Coordinate translateToCoordinates(const Eigen::Vector3d& vector) const {
      return Coordinate {
              static_cast<uint32_t>(vector(0) / resolution + width / 2),
              static_cast<uint32_t>(vector(1) / resolution + height / 2)
      };
    }

    //}

    /* translateFromCoordinates(Coordinate, z) -> Vector3 //{ */

    Eigen::Vector2d translateFromCoordinates(Coordinate coord, double z) const {
      return Eigen::Vector2d {
              (coord.x - width / 2.0) * resolution + resolution / 2,
              (coord.y - height / 2.0) * resolution + resolution / 2
      };
    }

    //}

    /* Manhattan Distnace Grid //{ */

    using manhattanDistance_t                                 = int8_t;
    static constexpr manhattanDistance_t manhattanDistanceMax = 100;
    using ManhattanDistanceGrid                               = Grid<manhattanDistance_t>;

    ManhattanDistanceGrid createManhattanDistanceGrid() const {
      ManhattanDistanceGrid distanceGrid{width, height, resolution, manhattanDistanceMax};

      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          if (this->operator()(y, x) > 20) {
            distanceGrid(y, x) = 0;
          }
          if (x > 0 && distanceGrid(y, x - 1) < distanceGrid(y, x)) {
            distanceGrid(y, x) = distanceGrid(y, x - 1) + 1;
          }
          if (y > 0 && distanceGrid(y - 1, x) < distanceGrid(y, x)) {
            distanceGrid(y, x) = distanceGrid(y - 1, x) + 1;
          }
        }
      }

      for (uint32_t y = height - 1;; --y) {
        for (uint32_t x = width - 1;; --x) {
          if (x < width - 1 && distanceGrid(y, x + 1) < distanceGrid(y, x)) {
            distanceGrid(y, x) = distanceGrid(y, x + 1) + 1;
          }
          if (y < height - 1 && distanceGrid(y + 1, x) < distanceGrid(y, x)) {
            distanceGrid(y, x) = distanceGrid(y + 1, x) + 1;
          }

          if (x == 0)
            break;
        }
        if (y == 0)
          break;
      }

      return distanceGrid;
    }

    //}

    /* updateDistanceBasedOnNeighbours() helper //{ */

    using distance_t                        = float;
    static constexpr distance_t distanceMax = __FLT_MAX__;
    using DistanceGrid                      = Grid<distance_t>;

    using ObstacleNumberGrid = Grid<Coordinate>;

private:

    // function to be used in specializations
    /* __updateDistnaceBasedOnNeighbours() //{ */

    template <bool updateObstacleGrid>
    void __updateDistanceBasedOnNeighbours(DistanceGrid& verticalDistance, DistanceGrid& horizontalDistance,
                                           ObstacleNumberGrid& obstacle, const int verticalDirection, const int horizontalDirection) const
    {
      uint32_t minX = horizontalDirection == 1 ? 0 : horizontalDistance.getWidth() - 1;
      uint32_t maxX = horizontalDirection == 1 ? horizontalDistance.getWidth() - 1 : 0;
      uint32_t minY = verticalDirection == 1 ? 0 : verticalDistance.getHeight() - 1;
      uint32_t maxY = verticalDirection == 1 ? verticalDistance.getHeight() - 1 : 0;

      uint32_t y = minY;
      while (true) {

        uint32_t x = minX;
        while (true) {
          float currentDistance2 = length2(verticalDistance(y, x), horizontalDistance(y, x));

          if (x != minX) {
            float distance2 = length2(verticalDistance(y, x - horizontalDirection),
                                      horizontalDistance(y, x - horizontalDirection) + horizontalDirection);
            if (distance2 < currentDistance2) {
              verticalDistance(y, x)   = verticalDistance(y, x - horizontalDirection);
              horizontalDistance(y, x) = horizontalDistance(y, x - horizontalDirection) + horizontalDirection;
              if (updateObstacleGrid) obstacle(y, x) = obstacle(y, x - horizontalDirection);
              currentDistance2 = distance2;
            }
          }

          if (y != minY) {
            float distance2 = length2(verticalDistance(y - verticalDirection, x) + verticalDirection,
                                      horizontalDistance(y - verticalDirection, x));
            if (distance2 < currentDistance2) {
              verticalDistance(y, x)   = verticalDistance(y - verticalDirection, x) + verticalDirection;
              horizontalDistance(y, x) = horizontalDistance(y - verticalDirection, x);
              if (updateObstacleGrid) obstacle(y, x) = obstacle(y - verticalDirection, x);
              currentDistance2 = distance2;
            }
          }

          if (x != minX && y != minY) {
            float distance2 = length2(verticalDistance(y - verticalDirection, x - horizontalDirection) + verticalDirection,
                                      horizontalDistance(y - verticalDirection, x - horizontalDirection) + horizontalDirection);
            if (distance2 < currentDistance2) {
              verticalDistance(y, x)   = verticalDistance(y - verticalDirection, x - horizontalDirection) + verticalDirection;
              horizontalDistance(y, x) = horizontalDistance(y - verticalDirection, x - horizontalDirection) + horizontalDirection;
              if (updateObstacleGrid) obstacle(y, x) = obstacle(y - verticalDirection, x - horizontalDirection);
            }
          }

          if (x == maxX)
            break;
          x += horizontalDirection;
        }

        if (y == maxY)
          break;
        y += verticalDirection;
      }
    }

    //}

    // specialization for createDistnaceGrid()
    void updateDistanceBasedOnNeighbours(DistanceGrid& verticalDistnace, DistanceGrid& horizontalDistance,
                                         const int verticalDirection, const int horizontalDirection) const
    {
      ObstacleNumberGrid obstacle = ObstacleNumberGrid{0, 0, 0};
      __updateDistanceBasedOnNeighbours<0>(verticalDistnace, horizontalDistance,
                                           obstacle, verticalDirection, horizontalDirection);
    }

    // specialization for createVoronoiDiagram()
    void updateDistanceBasedOnNeighbours(DistanceGrid& verticalDistnace, DistanceGrid& horizontalDistance,
                                         ObstacleNumberGrid& obstacle, const int verticalDirection, const int horizontalDirection) const
    {
      __updateDistanceBasedOnNeighbours<1>(verticalDistnace, horizontalDistance,
                                           obstacle, verticalDirection, horizontalDirection);
    }

public:

    //}

    /* createDistanceGrid() //{ */

    DistanceGrid createDistanceGrid() const {
      DistanceGrid verticalDistance{width, height, resolution, distanceMax};
      DistanceGrid horizontalDistance{width, height, resolution, distanceMax};

      // fill the obstacles
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          if (this->operator()(y, x) > 20) {
            verticalDistance(y, x) = horizontalDistance(y, x) = 0;
            continue;
          }
        }
      }

      // from top-left corner
      updateDistanceBasedOnNeighbours(verticalDistance, horizontalDistance, 1, 1);
      // from top-right corner
      updateDistanceBasedOnNeighbours(verticalDistance, horizontalDistance, -1, 1);
      // from bottom-right corner
      updateDistanceBasedOnNeighbours(verticalDistance, horizontalDistance, -1, -1);
      // from bottom-left corner
      updateDistanceBasedOnNeighbours(verticalDistance, horizontalDistance, 1, -1);

      // calculate the real distances and store them in verticalDistnace
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          verticalDistance(y, x) = std::sqrt(length2(verticalDistance(y, x), horizontalDistance(y, x)));
        }
      }

      return verticalDistance;
    }

    //}

    /* createVoronoiGraph() //{ */

    Grid<bool> createVoronoiGraph(uint32_t minSafeDistance, uint32_t maxCautionDistance) const
    {
      DistanceGrid verticalDistance{width, height, resolution, distanceMax};
      DistanceGrid horizontalDistance{width, height, resolution, distanceMax};
      ObstacleNumberGrid obstacle{width, height, resolution};

      // fill the obstacles
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          if (this->operator()(y, x) > 20) {
            verticalDistance(y, x) = horizontalDistance(y, x) = 0;
            obstacle(y, x) = Coordinate{x, y};
          }
        }
      }

      // from top-left corner
      updateDistanceBasedOnNeighbours(verticalDistance, horizontalDistance, obstacle, 1, 1);
      // from top-right corner
      updateDistanceBasedOnNeighbours(verticalDistance, horizontalDistance, obstacle, -1, 1);
      // from bottom-right corner
      updateDistanceBasedOnNeighbours(verticalDistance, horizontalDistance, obstacle, -1, -1);
      // from bottom-left corner
      updateDistanceBasedOnNeighbours(verticalDistance, horizontalDistance, obstacle, 1, -1);

      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          horizontalDistance(y, x) = sqr(verticalDistance(y, x)) + sqr(horizontalDistance(y, x));
        }
      }

      Grid<bool> result{width, height, resolution};

      float minSafeDistance2 = sqr(minSafeDistance);
      float maxCautionDistance2 = sqr(maxCautionDistance);
      for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
          if (horizontalDistance(y, x) < minSafeDistance2) {
            result(y, x) = 0;
            continue;
          }
          if (horizontalDistance(y, x) > maxCautionDistance2) {
            result(y, x) = 1;
            continue;
          }

          // to check if it is on the border of 2 obstacle regions
          auto isOnBorder = [&obstacle, &minSafeDistance2](uint32_t y1, uint32_t x1, uint32_t y2, uint32_t x2) {
              Coordinate obstacle1 = obstacle(y1, x1);
              Coordinate obstacle2 = obstacle(y2, x2);
              if (obstacle1.y == obstacle2.y ? obstacle1.x > obstacle2.x : obstacle1.y > obstacle2.y)
                return false;
              return sqr(obstacle1.x - obstacle2.x) + sqr(obstacle1.y - obstacle2.y) > 4 * minSafeDistance2;
          };

          if ((x != 0 && isOnBorder(y, x, y, x - 1)) || 
              (x != obstacle.getWidth() - 1 && isOnBorder(y, x, y, x + 1)) ||
              (y != 0 && isOnBorder(y, x, y - 1, x)) || 
              (y != obstacle.getHeight() - 1 && isOnBorder(y, x, y + 1, x))) {
            result(y, x) = 1;
            continue;
          }

          result(y, x) = 0;
        }
      }

      return result;
    }

    //}
    
  // getSubGridAtPosition(Vector3d, size) //{ 
  
  Grid<T> getSubGridAtPosition(Eigen::Vector3d& vector, double requiredDistance) const {
    uint32_t size = std::ceil(requiredDistance / resolution);
    // TODO use the grid coordinates to calculate this
    int64_t startX = (int64_t) std::round(width / 2.0 + (vector(0) - requiredDistance / 2) / resolution);
    int64_t startY = (int64_t) std::round(height / 2.0 + (vector(1) - requiredDistance / 2) / resolution);

    Coordinate min{startX < 0 ? 0 : (uint32_t) startX, startY < 0 ? 0 : (uint32_t) startY};
    Coordinate max{startX + size > width ? width : (uint32_t) startX + size, 
      startY + size > height ? height : (uint32_t) startY + size};

    if (startX >= width || startY >= height || startX + size <= 0 || startY + size <= 0) {
      return Grid<T>{0, 0, 0};
    } 

    Grid<T> result{max.x - min.x, max.y - min.y, resolution};
    for (uint32_t y = min.y; y < max.y; ++y) {
      for (uint32_t x = min.x; x < max.x; ++x) {
        result(y - min.y, x - min.x) = operator()(y, x);
      }
    }

    return result;
  }

  //}

  /* drawObstacleCircle() //{ */

  void drawObstacleCircle(Eigen::Vector3d position, double radius) {
    uint32_t radiusSize = std::ceil(radius / resolution);
    int64_t startX = (int64_t) std::round(width / 2.0 + (position(0) - radius / 2) / resolution);
    int64_t startY = (int64_t) std::round(height / 2.0 + (position(1) - radius / 2) / resolution);

    Coordinate min{startX < 0 ? 0 : (uint32_t) startX, startY < 0 ? 0 : (uint32_t) startY};
    Coordinate max{startX + radiusSize > width ? width : (uint32_t) startX + radiusSize, 
      startY + radiusSize > height ? height : (uint32_t) startY + radiusSize};
    
    int64_t centerX = (int64_t) std::round(width / 2.0 + position(0) / resolution);
    int64_t centerY = (int64_t) std::round(height / 2.0 + position(1) / resolution);

    for (uint32_t y = min.y; y < max.y; ++y) {
      for (uint32_t x = min.x; x < max.x; ++x) {
        if (std::pow(y - centerY, 2) + std::pow(x - centerX, 2) < std::pow(radiusSize, 2))
          operator()(y, x) = 100;
      }
    }
  }

  //}
};

typedef Grid<int8_t> ObstacleGrid;

/* scaleResolution() //{ */

static ObstacleGrid scaleResolution(ObstacleGrid original, uint32_t factor) {
  ObstacleGrid res{original.getHeight() / factor, original.getWidth() / factor, original.getResolution() * factor, 0};

  for (uint32_t y = 0; y < original.getHeight(); ++y) {
    for (uint32_t x = 0; x < original.getWidth(); ++x) {
      if (original(y, x) == 100)
        res(y / factor, x / factor) = 100;
    }
  }

  return res;
}

//}

/* scaleValues() //{ */

template <typename T, typename P>
Grid<P> scaleValues(Grid<T>& source, T sourceMin, T sourceMax, P targetMin, P targetMax) {
  Grid<P> target{source.getWidth(), source.getHeight(), source.getResolution()};

  for (uint32_t y = 0; y < source.getHeight(); ++y) {
    for (uint32_t x = 0; x < source.getWidth(); ++x) {
      double fraction = (double)(source(y, x) - sourceMin) / (sourceMax - sourceMin);
      T value = targetMin + fraction * (targetMax - targetMin);
      value = value > targetMax ? targetMax : value < targetMin ? targetMin : value;

      target(y, x) = value;
    }
  }

  return target;
}

//}

#endif //FOREST_PATH_FINDER_GRID_H
