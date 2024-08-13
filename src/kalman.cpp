#include "kalman.h"

KalmanOdom::Measurement::Matrix KalmanOdom::Measurement::matrix() const {
  Matrix m;
  m << leftDriveTravel.internal(), rightDriveTravel.internal(),
      vertTravel.internal(), horiTravel.internal(), thetaAccel.internal();
  return m;
}

KalmanOdom::ControlInput::Matrix KalmanOdom::ControlInput::matrix() const {
  Matrix m;
  m << leftVoltage.internal(), rightVoltage.internal();
  return m;
}

KalmanOdom::State& KalmanOdom::stateTransition(State& x, Time deltaT) {
  return x;
}

/**
 * @brief Algorithm for computing sigma points
 *
 * @tparam N Dimensionality of the state. 2n+1 weights will be generated.
 * @see
 * https://github.com/rlabbe/filterpy/blob/3b51149ebcff0401ff1e10bf08ffca7b6bbc4a33/filterpy/kalman/sigma_points.py#L24
 */
template <size_t N> class MerweScaledSigmaPoints {
  public:
    struct Config {
        float alpha, beta, kappa;
        std::function<Eigen::Matrix<float, N, 1>(
            Eigen::Matrix<float, N, 1>& lhs, Eigen::Matrix<float, N, 1>& rhs)>
            subtract;
        std::function<Eigen::Matrix<float, N, N>(Eigen::Matrix<float, N, N>&)>
            sqrt;
    };

    struct Weights {
        Eigen::Matrix<float, 2 * N + 1, 1> mean, covar;
    };
  private:
    Config conf;
    Weights weights;

    void initWeights() {
      float lambda = conf.alpha * conf.alpha * (N + conf.kappa) - N;
      weights.mean = weights.covar =
          Eigen::Matrix<float, N, 1>::Constant(1 / (2 * (N + lambda)));
      weights.mean(0) = weights.covar(0) = lambda / (N + lambda);
      weights.covar(0) += 1 - conf.alpha * conf.alpha + conf.beta;
    }
  public:
    MerweScaledSigmaPoints(Config config) : conf(config) {};
};