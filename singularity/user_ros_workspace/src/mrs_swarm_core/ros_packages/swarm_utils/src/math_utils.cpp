#include "swarm_utils/math_utils.h"

namespace swarm_utils {

  namespace math_utils {

    /* sampleRandVec() //{ */
    /**
     * @brief samples a real valued random vector from a multivariate normal distribution
     * The variables are assumed to be independent, so the covariance matrix has zero off-diagonal elements.
     *
     * @tparam rows dimension of multivariate distribution
     * @param mean mean of multivariate distribution
     * @param std_dev std dev matrix (must not have off-diagonal elements)
     *
     * @return sampled random vector
     */
    e::VectorXd sampleRandVec(const e::VectorXd& mean, const e::MatrixXd& std_dev) {

      std::random_device rd{};
      std::mt19937 rand_gen{rd()};
      std::normal_distribution<> nd{0,1};
      e::VectorXd ret(mean.rows());

      for (int row = 0; row < mean.rows(); row++) {
        ret(row) = nd(rand_gen);
      }

      return std_dev * ret + mean;
    }

    //}

  }
}
