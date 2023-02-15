#include "spline.h"

/* getFirstDerivativeAtPoints() //{ */

Eigen::MatrixXd getFirstDerivativeAtPoints(const Eigen::MatrixXd& points) {
  // This is part of building the cubic spline
  // Function takes points (x, y) as rows to build the spline
  // Returns derivatives for each of the segments, from which the curves can be reconstructed

  if (points.cols() != 2) {
    throw std::invalid_argument("[Cubic Spline] The number of cols needs to be 2");
  }
  if (points.rows() < 2) {
    throw std::invalid_argument("[Cubic Spline] At least 2 points needed");
  }

  int n = points.rows() - 1;
  Eigen::MatrixXd diagonal{n + 1, 1};
  Eigen::MatrixXd result{n + 1, 2};

  // fill the result (right part of the equation and diagonal)
  for (int i = 0; i < n + 1; ++i) {
    result.row(i) = 3 * (
            points.row(i == n ? n : i + 1) -
            points.row(i == 0 ? 0 : i - 1));
    diagonal(i) = (i == 0 || i == n ? 2 : 4);
  }

  // row elimination to row echelon form
  for (int i = 0; i < n; ++i) {
    // divide to get 1 on diagonal and subtract the row at the bottom
    result.row(i) /= diagonal(i);
    result.row(i + 1) -= result.row(i);
    diagonal(i + 1) -= 1 / diagonal(i);
  }
  result.row(n) /= diagonal(n);

  // reduced row echelon form
  for (int i = 0; i < n; ++i) {
    // Subtract the required amount from top to get a matrix with 1 on the diagonal
    result.row(i) -= result.row(i + 1) / diagonal(i);
  }

  return result;
}

//}

/* buildCubicSplinePath //{ */

Eigen::MatrixXd buildCubicSplinePath(Eigen::MatrixXd& points,
                                     const int numberInSection = 10)
{
  // get the derivatives
  Eigen::MatrixXd derivative = getFirstDerivativeAtPoints(points);

  int n = points.rows() - 1;
  Eigen::MatrixXd result{n * numberInSection + 1, 2};

  for (int i = 0; i < n; ++i) {
    // recalculate the coeficients
    Eigen::RowVector2d a = points.row(i);
    Eigen::RowVector2d b = derivative.row(i);
    Eigen::RowVector2d c = 3 * (points.row(i + 1) - points.row(i)) -
                           2 * derivative.row(i) - derivative.row(i + 1);
    Eigen::RowVector2d d = 2 * (points.row(i) - points.row(i + 1)) +
                           derivative.row(i) + derivative.row(i + 1);

    for (int k = 0; k < numberInSection; ++k) {
      float t = (float) k / numberInSection;
      // a + bt + ct^2 + dt^3
      result.row(i * numberInSection + k) = a + (b + (c + d * t) * t) * t;
    }
  }
  result.row(result.rows() - 1) = points.row(points.rows() - 1);

  return result;
}

//}

