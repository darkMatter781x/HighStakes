#include "kalman/odom.h"
#include "Eigen/src/Core/Matrix.h"

KalmanOdom::KalmanOdom(const Config config)
  : m_processModel(config.processNoise), m_stateOps(),
    m_ukf(UKF::Config {
        .sigmas = config.sigmas,
        .initialState = config.initialState,
        .initialCovar = config.initialCovar,
        .stateOps = m_stateOps,
    }),
    m_mean(State {.m_vector = m_ukf.m_mean}), m_covar(m_ukf.m_covar),
    m_time(0) {}

kalman::State<KalmanOdom::N> KalmanOdom::predict(Time t) const {
  return {m_ukf.predict(m_processModel, (t - m_time).internal()).mean};
}

KalmanOdom::ProcessModel::ProcessModel(Eigen::Matrix<float, N, N> noise)
  : noise(noise) {}

Eigen::Vector<float, KalmanOdom::N>
KalmanOdom::ProcessModel::predict(const Eigen::Vector<float, N>& inVec,
                                  float dT) const {
  const State in {const_cast<Eigen::Vector<float, N>&>(inVec)};
  Eigen::Vector<float, N> outVec;
  State out {outVec};

  const float dT2 = dT * dT;
  out.xRef() = in.xRef() + dT * in.xVelRef() + dT2 * .5 * in.xAccelRef();
  out.xVelRef() = in.xVelRef() + dT * in.xAccelRef();
  out.xAccelRef() = in.xAccelRef();
  out.yRef() = in.yRef() + dT * in.yVelRef() + dT2 * .5 * in.yAccelRef();
  out.yVelRef() = in.yVelRef() + dT * in.yAccelRef();
  out.yAccelRef() = in.yAccelRef();
  out.thetaRef() = in.thetaRef() + dT * in.thetaVelRef() + dT2 * .5 * in.thetaAccelRef();
  out.thetaVelRef() = in.thetaVelRef() + dT * in.thetaAccelRef();
  out.thetaAccelRef() = in.thetaAccelRef();

  return outVec;
}

Eigen::Matrix<float, KalmanOdom::N, 1> KalmanOdom::StateOps::mean(
    const Eigen::Matrix<float, N, UKF::SIGMA_N>& sigma_points,
    const Eigen::Matrix<float, UKF::SIGMA_N, 1>& weights) const {
  return sigma_points * weights;
}

Eigen::Matrix<float, KalmanOdom::N, 1>
KalmanOdom::StateOps::diff(const Eigen::Vector<float, N>& sigma_points,
                           const Eigen::Vector<float, N>& weights) const {
  return sigma_points - weights;
}