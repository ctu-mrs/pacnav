#ifndef FOREST_PATH_FINDER_ASTAR_H
#define FOREST_PATH_FINDER_ASTAR_H

#include <vector>
#include "path_finder/grid.h"

std::vector<Coordinate> aStar (const Grid<bool>& grid, const Coordinate start, const Coordinate finish);

#endif //FOREST_PATH_FINDER_ASTAR_H
