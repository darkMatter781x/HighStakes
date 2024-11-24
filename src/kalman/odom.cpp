#include "kalman/odom.h"
#include "pros/rtos.hpp"
#include "sigma_points.h"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>

using Odom = KalmanOdom;

Odom::KalmanOdom(const Config config)
  : m_processModel(config.processNoise), m_stateOps(),
    m_ukf(UKF::Config {
        .sigmas = config.sigmas,
        .initialState = config.initialState,
        .initialCovar = config.initialCovar,
        .stateOps = m_stateOps,
    }),
    m_mean(State {.m_vector = m_ukf.m_mean}), m_covar(m_ukf.m_covar),
    m_time(pros::millis() * msec) {}

kalman::State<Odom::N> Odom::predict(Time t) const {
  return m_ukf.predictConst(m_processModel, (t - m_time).internal());
}

Odom::ProcessModel::ProcessModel(Eigen::Matrix<float, N, N> noise)
  : noise(noise) {}

Eigen::Vector<float, Odom::N>
Odom::ProcessModel::predict(const Eigen::Vector<float, N>& inVec,
                            float dT) const {
  /** input state */
  const State::Refs i = State::fromConst(inVec).r;
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

Eigen::Matrix<float, Odom::N, 1> Odom::StateOps::mean(
    const Eigen::Matrix<float, N, UKF::SIGMA_N>& sigma_points,
    const Eigen::Matrix<float, UKF::SIGMA_N, 1>& weights) const {
  return sigma_points * weights;
}

Eigen::Matrix<float, Odom::N, 1>
Odom::StateOps::diff(const Eigen::Vector<float, N>& sigma_points,
                     const Eigen::Vector<float, N>& weights) const {
  return sigma_points - weights;
}

void Odom::print() const {
  std::cout << "t: " << m_time << std::endl;
  std::cout << "\tmean: " << m_mean.m_vector << std::endl;
  std::cout << "\tcovar: " << m_covar << std::endl;
}

class TrackingWheelModel : Odom::UKF::MeasurementModel<2> {
  public:
    /** measurement vector order: left, right */
    struct Config {
        /** order: left, right */
        Eigen::Vector<float, M> offsets;
        Eigen::Matrix<float, M, M> noise;
    };

    Eigen::Vector<float, M>
    function(const Eigen::Vector<float, N>& mean) override {
      using namespace units;
      const Odom::State state = Odom::State::fromConst(mean);
      float num =
          state.r.x<1>() * state.r.x<1>() + state.r.y<1>() * state.r.y<1>();
      float denom = 2 - 2 * cos(state.r.theta<1>());
      float radius = sqrt(num / denom);
      if (std::isnan(radius) || std::isinf(radius)) return {0, 0};

      float left = (radius + m_offsets(0)) * state.r.theta<1>();
      float right = (radius + m_offsets(1)) * state.r.theta<1>();
      return {left, right};
    };

    Eigen::Matrix<float, M, M> noise;

    /** default */
    kalman::UT::Operators<M, Odom::UKF::SIGMA_N> ops;

    static Odom::Measurement<M> toMeasurement(TrackingWheelModel& model,
                                              Eigen::Vector<float, M> vector) {
      return {.vector = vector, .model = model};
    };
  private:
    Eigen::Vector<float, M> m_offsets;
  public:
    TrackingWheelModel(Config config)
      : noise(config.noise), m_offsets(config.offsets) {};
};

using namespace units;

V2Position lerp(V2Position a, V2Position b, float t) {
  return V2Position {
      std::lerp(float(a.getX().internal()), float(b.getX().internal()), t) * m,
      std::lerp(float(a.getY().internal()), float(b.getY().internal()), t) * m};
}

std::vector<Pose> makePath(V2Position start, V2Position end, size_t points,
                           Angle theta) {
  std::vector<Pose> vec;
  for (size_t i = 0; i < points; i++) {
    vec.push_back({lerp(start, end, float(i) / (points - 1)), theta});
  }
  return vec;
}

// kalman::MerweSigmaPts<Odom::N> makeSigmaPts() {
//   return kalman::MerweSigmaPts<Odom::N>::Config {
//       .alpha = 0.1,
//       .beta = 2.0,
//       .kappa = 0.0,
//       .sqrt =
//           [](const Eigen::Matrix<float, Odom::N, Odom::N>& mat) {
//             return mat.llt().matrixL();
//           },
//   };
// }
Eigen::Matrix<float, Odom::N, Odom::N> sqrtCovarMatrix(const Eigen::Matrix<float, Odom::N, Odom::N>& mat) {
  return mat.llt().matrixL();
}

void kalmanTestingMain() {
  pros::delay(500);
  std::cout << "[[Kalman Testing Start]]" << std::endl;

  Pose start {0.0_m, 0.0_m, 0 * rad};
  Pose end {0.0_m, 1.0_m, 0 * rad};
  std::vector<Pose> path = makePath(start, end, 100, 0 * rad);

  TrackingWheelModel::Config twConfig {
      .offsets = {-0.1, 0.1},
      .noise = Eigen::Matrix<float, 2, 2>::Identity()};
  TrackingWheelModel twModel {twConfig};

  Odom::Config config {
      .sigmas {
          .alpha = 0.1,
          .beta = 2.0,
          .kappa = -6.0,
          .sqrt = sqrtCovarMatrix,
      },
      .processNoise = Eigen::Matrix<float, Odom::N, Odom::N>::Identity(),
      .initialState = Eigen::Vector<float, Odom::N>::Zero(),
      .initialCovar = Eigen::Matrix<float, Odom::N, Odom::N>::Identity(),
  };

  const Eigen::Matrix<float, Odom::N, Odom::N> A =
      Eigen::Matrix<float, Odom::N, Odom::N>::Identity();

  Eigen::Matrix<float, Odom::N, Odom::N> L = sqrtCovarMatrix(A);

  std::cout << "L: " << L << std::endl;
  std::cout << "L L^T: " << L * L.transpose() << std::endl;

  Odom odom {config};
  odom.m_mean.r.x() = start.getX().internal();
  odom.m_mean.r.y() = start.getY().internal();
  odom.m_mean.r.theta() = start.theta().internal();

  for (int i = 0; i < path.size(); i++) {
    // predict step
    printf("predict:\n");

    Time t = odom.m_time + 10_msec;
    kalman::State prior = odom.predict(t);
    odom.m_time = t;
    odom.m_mean.m_vector = prior.mean;
    odom.m_covar = prior.covar;

    odom.print();

    // update step
    printf("update:\n");

    Pose current = path[i];

    Eigen::Vector<float, 2> z = twModel.function(odom.m_mean.m_vector);
    std::cout << "z: " << z << std::endl;

    odom.update<2>(TrackingWheelModel::toMeasurement(twModel, z));
    odom.print();
  }
}