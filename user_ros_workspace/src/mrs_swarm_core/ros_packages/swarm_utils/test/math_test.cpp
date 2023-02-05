#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch2_tester.h"
#include "swarm_utils/math_utils.h"

/* sample N-dim rand vec //{ */

TEST_CASE("sample N-dim rand vec", "[sample_rand_vec]") {

  e::Vector3d mean{0, 0, 0};
  e::Matrix<double, 3, 3> std_dev;
  std_dev <<
    1.0, 0, 0,
    0, 1.0, 0,
    0, 0, 1.0;

  e::Vector3d rand_vec = swarm_utils::math_utils::sampleRandVec(mean, std_dev);
  REQUIRE(rand_vec.size() == mean.size());

  const int s_size = 1000;
  const double eps = 0.1;
  e::Vector3d s_mean{0, 0, 0};
  e::Matrix<double, 3, 3> s_std_dev;
  s_std_dev <<
    0, 0, 0,
    0, 0, 0,
    0, 0, 0;
  e::Matrix<double, 3, s_size> s_vecs;

  for (int it = 0; it < s_size; it++) {
    s_vecs.col(it) = swarm_utils::math_utils::sampleRandVec(mean, std_dev);
    s_mean += s_vecs.col(it);
  }
  s_mean = s_mean / s_size;

  REQUIRE((s_mean - mean).norm() < eps);

  for (int it = 0; it < (s_size - 1); it++) {
    s_std_dev += s_vecs.col(it) * s_vecs.col(it).transpose();
  }
  s_std_dev = s_std_dev / s_size;

  for (int it = 0; it < mean.size(); it++) {
    for (int it2 = 0; it2 < mean.size(); it2++) {
      REQUIRE(std::abs(s_std_dev(it, it2) - std_dev(it, it2)) < eps);
    }
  }
}

//}

