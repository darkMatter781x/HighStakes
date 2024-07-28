/**
 * @brief Kalman filter for odometry (pls work)
 */
#pragma once
#include "Eigen/Core"
#include "Eigen/src/Core/Matrix.h"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

class KalmanOdom {
  private:
    static constexpr uint STATE_VARS = 9;
    static constexpr uint MEASUREMENT_VARS = 5;
  public:
    struct State {
      public:
        /** x, x', x'', y, y', y'', theta, theta', theta'' */
        Eigen::Matrix<float, STATE_VARS, 1> m_matrix;
        Time m_lastTime;
      private:
        template <uint Index, isQuantity Q> inline Q get(Q q) const {
          static_assert(Index < STATE_VARS, "");
          return m_matrix(Index) * q;
        }
      public:
        inline Length x() const { return get<0>(cm); }

        inline LinearVelocity xVel() const { return get<1>(cmps); }

        inline LinearAcceleration xAccel() const { return get<2>(cmps2); }

        inline Length y() const { return get<3>(cm); }

        inline LinearVelocity yVel() const { return get<4>(cmps); }

        inline LinearAcceleration yAccel() const { return get<5>(cmps2); }

        inline Angle theta() const { return get<6>(deg); }

        inline AngularVelocity thetaVel() const { return get<7>(degps); }

        inline AngularAcceleration thetaAccel() const { return get<8>(degps2); }

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

    typedef Eigen::Matrix<float, STATE_VARS, STATE_VARS> Covariance;

    struct Measurement {
        typedef Eigen::Matrix<float, MEASUREMENT_VARS, 1> Matrix;
        LinearVelocity leftDriveTravel;
        LinearVelocity rightDriveTravel;
        LinearVelocity vertTravel;
        LinearVelocity horiTravel;
        AngularAcceleration thetaAccel;

        Matrix matrix() const;
    };

    struct ControlInput {
        typedef Eigen::Matrix<float, 2, 1> Matrix;
        Voltage leftVoltage;
        Voltage rightVoltage;

        Matrix matrix() const;
    };
  private:
    /** @symbol bold x */
    State m_state;
    /** @symbol bold P */
    Covariance m_covariance;
  public:
    const State& getState() const { return m_state; }

    /**
     * @brief represents the noise in the process model
     * @symbol bold Q
     */
    static Covariance process_noise;

    /**
     * @brief transforms state into measurement space
     * @symbol bold H
     */
    static Eigen::Matrix<float, MEASUREMENT_VARS, STATE_VARS>
        measurement_function;
    /**
     * @brief computes prior
     * @symbol bold F
     */
    static State& stateTransition(State& x, Time deltaT);
    /**
     * @brief models effect of control input on state
     * @symbol bold B
     */
    static State& controlFunction(ControlInput& u, Time deltaT);

    /**
     * @return prediction of state at time t, based on m_state and u
     * @symbol bold x̄ (prior)
     */
    State predict(Time t, ControlInput u) const;

    /** @return updated state after incorporating measurement */
    State update(const Measurement& m) const;
};