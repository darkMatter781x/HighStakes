#include "kalman/odom.h"

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
  return {m_ukf.predictConst(m_processModel, (t - m_time).internal()).mean};
}

KalmanOdom::ProcessModel::ProcessModel(Eigen::Matrix<float, N, N> noise)
  : noise(noise) {}

Eigen::Vector<float, KalmanOdom::N>
KalmanOdom::ProcessModel::predict(const Eigen::Vector<float, N>& inVec,
                                  float dT) const {
  /** input state */
  const State::Refs i {const_cast<Eigen::Vector<float, N>&>(inVec)};
  Eigen::Vector<float, N> outVec;
  /** output state */
  State::Refs o {outVec};

  const float dT2 = dT * dT;
  o.x<0>() = i.x<0>() + dT * i.x<1>() + dT2 * .5 * i.x<2>();
  o.x<1>() = i.x<1>() + dT * i.x<2>();
  o.x<2>() = i.x<2>();
  o.y<0>() = i.y<0>() + dT * i.y<1>() + dT2 * .5 * i.y<2>();
  o.y<1>() = i.y<1>() + dT * i.y<2>();
  o.y<2>() = i.y<2>();
  o.theta<0>() = i.theta<0>() + dT * i.theta<1>() + dT2 * .5 * i.theta<2>();
  o.theta<1>() = i.theta<1>() + dT * i.theta<2>();
  o.theta<2>() = i.theta<2>();

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