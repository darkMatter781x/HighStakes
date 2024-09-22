/**
 * @brief Uses UKF (unscented kalman filter) for odometry (pls work)
 */
#pragma once

#include "kalman/ukf.h"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

template <isQuantity Q, size_t Deriv> using TimeDerivative =
    Divided<Q, Exponentiated<Time, std::ratio<Deriv>>>;

class KalmanOdom {
  public:
    static constexpr uint N = 9;
    using UKF = kalman::UKF<N>;
  protected:
    UKF m_ukf;
  public:
    struct State {
      public:
        /** x, x', x'', y, y', y'', theta, theta', theta'' */
        Eigen::Vector<float, N>& m_vector;

        struct Refs {
          private:
            Eigen::Vector<float, N>& m_vector;

            template <size_t Index, size_t Deriv = 0>
            inline float& getRef() const {
              static_assert(Index < N, "");
              static_assert(0 <= Deriv && Deriv <= 2,
                            "Deriv must be 0, 1, or 2");
              return m_vector(Index + Deriv);
            }
          public:
            template <size_t Deriv = 0> inline float& x() {
              return getRef<0, Deriv>();
            }

            template <size_t Deriv = 0> const inline float& x() const {
              return getRef<0, Deriv>();
            }

            template <size_t Deriv = 0> inline float& y() {
              return getRef<3, Deriv>();
            }

            template <size_t Deriv = 0> const inline float& y() const {
              return getRef<3, Deriv>();
            }

            template <size_t Deriv = 0> inline float& theta() {
              return getRef<6, Deriv>();
            }

            template <size_t Deriv = 0> const inline float& theta() const {
              return getRef<6, Deriv>();
            }

            inline Refs(Eigen::Vector<float, N>& vector) : m_vector(vector) {}
        } r {m_vector};

        template <size_t Deriv = 0>
        inline TimeDerivative<Length, Deriv> x() const {
          return r.x<Deriv>() * TimeDerivative<Length, Deriv> {1.0};
        }

        template <size_t Deriv = 0>
        inline TimeDerivative<Length, Deriv> y() const {
          return r.y<Deriv>() * TimeDerivative<Length, Deriv> {1.0};
        }

        template <size_t Deriv = 0>
        inline TimeDerivative<Angle, Deriv> theta() const {
          return r.theta<Deriv>() * TimeDerivative<Angle, Deriv> {1.0};
        }

        template <size_t Deriv = 0>
        inline units::Vector2D<TimeDerivative<Length, Deriv>> vec() const {
          return {x<Deriv>(), y<Deriv>()};
        }

        template <size_t Deriv = 0>
        inline units::AbstractPose<std::ratio<Deriv>> pose() const {
          return {vec<Deriv>(), theta<Deriv>()};
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
             const Eigen::Matrix<float, UKF::SIGMA_N, 1>& weights)
            const override;
        Eigen::Matrix<float, N, 1>
        diff(const Eigen::Vector<float, N>& lhs,
             const Eigen::Vector<float, N>& rhs) const override;
    };

    template <size_t __M> struct Measurement {
        static constexpr size_t M = __M;
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
