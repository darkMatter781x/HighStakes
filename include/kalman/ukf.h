#pragma once

#include "Eigen/Dense"
#include "kalman/sigma_points.h"
#include "kalman/unscented_transform.h"
#include "kalman/util.h"

namespace kalman {

template <size_t __N> class UnscentedKalmanFilter {
  public:
    static constexpr size_t N = __N;
    using SigmaPts = MerweSigmaPts<N>;
    static constexpr size_t SIGMA_N = SigmaPts::SIGMA_N;

    class ProcessModel {
      public:
        /**
         * @brief Projects the state mean forward in time.
         * @symbol F
         *
         * @param mean The state mean.
         * @param dt How far forward in time to project the state mean.
         */
        virtual Eigen::Matrix<float, N, 1>
        predict(const Eigen::Matrix<float, N, 1>& mean, float dt) const = 0;
        /**
         * @brief Process noise.
         * @symbol Q
         */
        Eigen::Matrix<float, N, N> noise;
    };

    using F = ProcessModel;

    /**
     * @brief Converts a state mean to measurement space.
     *
     * @tparam M Dimensionality of measurement
     */
    template <size_t __M> class MeasurementModel {
      public:
        static constexpr size_t M = __M;
        static constexpr size_t N = __N;
        using SigmaPts = MerweSigmaPts<M>;
        static constexpr size_t SIGMA_M = SigmaPts::SIGMA_N;

        /**
         * @brief Converts state mean to measurement space.
         * @symbol H
         */
        virtual Eigen::Matrix<float, M, 1>
        function(const Eigen::Matrix<float, N, 1>& mean) = 0;

        /**
         * @brief Measurement noise.
         * @symbol R
         */
        Eigen::Matrix<float, M, M> noise;

        /**
         * @brief Measurement sigma point operators.
         */
        UT::Operators<M, SIGMA_M> ops;
    };

    template <size_t M> using H = MeasurementModel<M>;

    struct Config {
        SigmaPts::Config sigmas;
        Eigen::Matrix<float, N, 1> initialState;
        Eigen::Matrix<float, N, N> initialCovar;
        UT::Operators<N, SIGMA_N>& stateOps;
    };

    /**
     * @brief The state mean.
     * @symbol x
     */
    Eigen::Matrix<float, N, 1> m_mean;

    /**
     * @brief The state covariance.
     * @symbol P
     */
    Eigen::Matrix<float, N, N> m_covar;
  private:
    /**
     * @brief Internal function for prediction. Updates the sigmas_f matrix.
     *
     * @param model Model to use for prediction.
     * @param dt Delta time; how far to predict forward in time.
     * @param sigmas_f The matrix to store the sigma points in.
     * @return State<N> The new state.
     */
    template <
        bool MUT_SIGMAS,
        typename __Sigmas = std::conditional<
            MUT_SIGMAS, Eigen::Matrix<float, N, SIGMA_N>&, std::nullptr_t>>
    State<N> _predict(const ProcessModel& model, float dt,
                      __Sigmas sigmas_f_raw) const {
      Eigen::Matrix<float, N, SIGMA_N>* sigmas_f;
      if constexpr (MUT_SIGMAS) {
        sigmas_f = &sigmas_f_raw;
      } else {
        Eigen::Matrix<float, N, SIGMA_N> sigmas_f_storage;
        sigmas_f = &sigmas_f_storage;
      }

      Eigen::Matrix<float, N, SIGMA_N> sigmas =
          m_sigmas.computeSigmaPoints(m_mean, m_covar);
      for (size_t i = 0; i < SIGMA_N; i++) {
        sigmas_f->col(i) = model.predict(sigmas.col(i), dt);
      }

      State newState =
          UT::transform<N, SIGMA_N>(*sigmas_f, m_sigmas.m_weights, m_stateOps);
      return newState;
      return {};
    }
  public:
    /**
     * @brief Perform prediction step, updating the state estimate.
     *
     * @param model Model to use for prediction.
     * @param dt Delta time; how far to predict forward in time.
     */
    void predict(const ProcessModel& model, float dt) {
      State newState = _predict(model, dt, m_sigmas_f);
      m_mean = newState.mean;
      m_covar = newState.covar;
    }

    /**
     * @brief Predict the state without updating the internal state.
     *
     * @param model Model to use for prediction.
     * @param dt Delta time; how far to predict forward in time.
     */
    State<N> predictConst(const ProcessModel& model, float dt) const {
      return _predict<false>(model, dt, nullptr);
    }

    /**
     * @brief Update the state estimate based on a measurement.
     *
     * @tparam M Dimensionality of measurement.
     * @param function Function to convert state means to measurement space.
     * @param measurement The actual measurement.
     */
    template <size_t M> void update(MeasurementModel<M>& model,
                                    Eigen::Matrix<float, M, 1>& measurement) {
      Eigen::Matrix<float, M, SIGMA_N> sigmas_h;
      for (size_t i = 0; i < SIGMA_N; i++) {
        Eigen::Vector<float, N> a = model.function(m_sigmas_f.col(i));
        sigmas_h.col(i) = a;
      }
      State expected_measurement =
          UT::transform(sigmas_h, m_sigmas.weights, model.ops);
      /** @symbol mu_z */
      auto mean_z = expected_measurement.mean;
      /** @symbol P_z */
      auto covar_z = expected_measurement.mean;
      /** @symbol P_xz */
      auto covar_xz = crossCovariance(m_sigmas_f, sigmas_h, m_mean, mean_z,
                                      m_sigmas.weights.covar);
      /** @symbol K */
      auto kalmanGain = covar_z * covar_xz.inverse();
      m_mean += kalmanGain * (measurement - mean_z);
      m_covar -= kalmanGain * covar_z * kalmanGain.transpose();
    }

    /**
     * @brief Construct a new Unscented Kalman Filter object.
     *
     * @param config Configuration for the UKF.
     */
    UnscentedKalmanFilter(const Config config);
  private:
    /** @brief The sigma points for the state. */
    Eigen::Matrix<float, N, SIGMA_N> m_sigmas_f;

    /** @brief Algorithm for generating sigma points. */
    const SigmaPts m_sigmas;
    const UT::Operators<N, SIGMA_N> m_stateOps;
};

template <size_t N> using UKF = UnscentedKalmanFilter<N>;

} // namespace kalman