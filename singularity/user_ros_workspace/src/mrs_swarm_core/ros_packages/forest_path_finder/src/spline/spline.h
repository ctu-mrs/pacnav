#ifndef FOREST_PATH_FINDER_SPLINE_H
#define FOREST_PATH_FINDER_SPLINE_H

#include <eigen3/Eigen/Eigen>

Eigen::MatrixXd buildCubicSplinePath(Eigen::MatrixXd& points,
                                     const int numberInSection);

#endif //FOREST_PATH_FINDER_SPLINE_H
