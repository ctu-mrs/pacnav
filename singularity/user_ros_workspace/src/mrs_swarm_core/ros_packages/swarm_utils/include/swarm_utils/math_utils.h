#pragma once
#ifndef MATH_UTILS_H
#define MATH_UTILS_H

/* for mathematical operations */
#include <cmath>
#include <random>

#include <eigen3/Eigen/Eigen>

namespace e = Eigen;

namespace swarm_utils {

  namespace math_utils {

    e::VectorXd sampleRandVec(const e::VectorXd& mean, const e::MatrixXd& std_dev);

  }
}

#endif
