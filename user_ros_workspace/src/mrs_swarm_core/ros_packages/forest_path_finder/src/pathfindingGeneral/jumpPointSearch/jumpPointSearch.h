#ifndef FOREST_PATH_FINDER_JUMP_POINT_SEARCH_H
#define FOREST_PATH_FINDER_JUMP_POINT_SEARCH_H

#include <vector>
#include "path_finder/grid.h"

std::vector<Coordinate> jumpPointSearch(const Grid<bool>& grid, const Coordinate start, const Coordinate finish);

#endif
