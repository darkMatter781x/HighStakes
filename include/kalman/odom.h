/**
 * @brief Uses UKF (unscented kalman filter) for odometry (pls work)
 */
#pragma once

#include "kalman/ukf.h"
#include "kalman/unscented_transform.h"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

#define __KALMAN_ODOM_INTERNAL_STATE_COEFF(name, upper_name, i, quantity,      \
                                           type)                               \
  inline type name() const { return get<i>(quantity); }                        \
  inline const float& name##Ref() const { return getRef<i>(); }                \
  inline float& name##Ref() { return getRef<i>(); }                            \
  inline void set##upper_name(type value) { getRef<i>() = value.internal(); }

#define __KALMAN_ODOM_INTERNAL_STATE_COEFFS(name, upper_name, i, q, q_prime,   \
                                            q_prime2, t, t_prime, t_prime2)    \
  __KALMAN_ODOM_INTERNAL_STATE_COEFF(name, upper_name, i, q, t)                \
  __KALMAN_ODOM_INTERNAL_STATE_COEFF(name##Vel, upper_name##Vel, i + 1,        \
                                     q_prime, t_prime)                         \
  __KALMAN_ODOM_INTERNAL_STATE_COEFF(name##Accel, upper_name##Accel, i + 2,    \
                                     q_prime2, t_prime2)

class KalmanOdom {
  protected:
    static constexpr uint N = 9;

    using UKF = kalman::UKF<N>;
    UKF m_ukf;
  public:
    struct State {
      public:
        /** x, x', x'', y, y', y'', theta, theta', theta'' */
        Eigen::Vector<float, N>& m_vector;
      private:
        template <uint Index> inline float& getRef() const {
          static_assert(Index < N, "");
          return m_vector(Index);
        }

        template <uint Index, isQuantity Q> inline Q get(Q q) const {
          static_assert(Index < N, "");
          return getRef<Index>() * q;
        }
      public:
        __KALMAN_ODOM_INTERNAL_STATE_COEFFS(x, X, 0, m, mps, mps2, Length,
                                            LinearVelocity, LinearAcceleration)
        __KALMAN_ODOM_INTERNAL_STATE_COEFFS(y, Y, 0, m, mps, mps2, Length,
                                            LinearVelocity, LinearAcceleration)
        __KALMAN_ODOM_INTERNAL_STATE_COEFFS(theta, Theta, 0, rad, radps, radps2,
                                            Angle, AngularVelocity,
                                            AngularAcceleration)

        inline units::V2Position vec() const { return {x(), y()}; }

        inline units::V2Velocity vecVel() const { return {xVel(), yVel()}; }

        inline units::V2Acceleration vecAccel() const {
          return {xAccel(), yAccel()};
        }

        inline units::Pose pose() const { return {vec(), theta()}; }

        inline units::VelocityPose poseVel() const {
          return {vecVel(), thetaVel()};
        }

        inline units::AccelerationPose poseAccel() const {
          return {vecAccel(), thetaAccel()};
        }
    };

    class ProcessModel : public UKF::ProcessModel {
      public:
        Eigen::Vector<float, N> predict(const Eigen::Vector<float, N>& x,
                                        float deltaT) const override;
        Eigen::Matrix<float, N, N> noise;

        ProcessModel(Eigen::Matrix<float, N, N> noise);
    };

    struct StateOps : kalman::UT::Operators<N, UKF::SIGMA_N> {
        Eigen::Matrix<float, N, 1>
        mean(const Eigen::Matrix<float, N, UKF::SIGMA_N>& sigma_points,
             const Eigen::Matrix<float, UKF::SIGMA_N, 1>& weights) override;
        Eigen::Matrix<float, N, 1>
        diff(const Eigen::Vector<float, N>& lhs,
             const Eigen::Vector<float, N>& rhs) override;
    };

    template <size_t M> class Measurement {
        /** @brief  (sensor data) */
        Eigen::Vector<float, M> vector;
        UKF::MeasurementModel<M>& model;
    };

    struct Config {
        UKF::SigmaPts::Config sigmas;
        Eigen::Matrix<float, N, N> processNoise;
        Eigen::Matrix<float, N, 1> initialState;
        Eigen::Matrix<float, N, N> initialCovar;
    };

    /** @symbol bold x */
    State m_mean;
    /** @symbol bold P */
    Eigen::Matrix<float, N, N>& m_covar;
    /** timestamp of last update to state */
    Time m_time;

    /**
     * @return prediction of state at time t, based on m_state and u
     * @symbol bold x̄ (prior)
     */
    kalman::State<N> predict(Time t) const;

    /** @return updated state after incorporating measurement */
    template <size_t M> void update(const Measurement<M>& m);

    KalmanOdom(const Config config);
  private:
    ProcessModel m_processModel;
    StateOps m_stateOps;
};

template <size_t M> void KalmanOdom::update(const Measurement<M>& m) {
  m_ukf.update(m.model, m.vector);
}

void kalmanTestingMain();
