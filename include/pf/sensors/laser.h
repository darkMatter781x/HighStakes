#pragma once

#include "Eigen/Core"

#include "Eigen/src/Core/Array.h"
#include "Eigen/src/Core/Matrix.h"
#include "dimensions.h"
#include "lemlib/pose.hpp"
#include "pf/sensors/sensor.h"
#include "pf/util/lines.h"
#include "pf/util/stats.h"
#include "units/units.hpp"
#include "math.h"
#include <optional>

namespace sensors {

/** Sensor model for range sensors */
namespace range_model {
namespace _impl {
constexpr auto square(auto a) { return a * a; }

auto cwiseErf(auto x) {
  return x.unaryExpr([](auto y) { return erf(y); });
}

auto normPDF(auto mu, auto sigma, auto x) {
  return 1 / (sqrt(2 * M_PI) * sigma) *
         exp(-1. / 2 * square(mu - x) / square(sigma));
};
} // namespace _impl

namespace density {
namespace areas {
constexpr Area ring = _impl::square(7. / 2 * in) * M_PI;
constexpr Area mogo = _impl::square(.42 * in) * M_PI;
constexpr Area wallStake = _impl::square(.42 * in) * M_PI;
constexpr Area fullMogo = ring;
constexpr Area robot = _impl::square(18 * in);
constexpr Area ladderPost = 1. / 2 * (7. / 2 * in) * (7. / 2 * in);
constexpr Area field = _impl::square(dimensions::field::FIELD_LENGTH * in);
constexpr Area ringStack = ring;

/** Assumes worst case during a match. */
constexpr Area untrackedObstaclesMatch =
    mogo * 5 + robot * 3 + ladderPost * 4 + ringStack * 4 + wallStake * 4;
/** Average over course of 1 minute skills run. */
constexpr Area untrackedObstaclesSkills =
    fullMogo * 3. / 2 + mogo * (2 + 3. / 2) + ladderPost * 4 + wallStake * 4;
/** I dont have all the elements :(. */
constexpr Area untrackedObstaclesLazySkills = ladderPost * 4;

/** Total area of the field, not including our robot. */
constexpr Area totalNotIncludingMe = field - robot;

} // namespace areas

/** Density of untracked obstacles during a match. Assumes worst case. */
constexpr double untrackedObstaclesMatch =
    (areas::untrackedObstaclesMatch / areas::totalNotIncludingMe).internal();
/** Density of untracked obstacles during a 1 minute skills run. Attempts to
 * use average density over time. */
constexpr double untrackedObstaclesSkills =
    (areas::untrackedObstaclesSkills / areas::totalNotIncludingMe).internal();

}; // namespace density

/** @brief Parameters for the range sensor model */
struct Parameters {
    /** Probability of a measurement that resulted from uniform measurement
     * noise. */
    float alpha_rand;

    /** Probability of a measurement that resulted from random measurement
     * errors. */
    float alpha_maxMiss;

    /** Standard deviation of measurements that resulted from hitting mapped
     * obstacles. Assumes such measurements are normally distributed. */
    float sigma_hit;

    /** Rate of hitting an obstacle for every given that the measurement
     * resulted from hitting an unmapped obstacle. Density of unmapped obstacles
     * seems to be a good heuristic. */
    float lambda_short;

    /** Value that represents a measurement error, often because the measurement
     * was outside the sensor's measurable range. */
    float z_max;
};

/**
 * @brief
 *
 * @param z The actual measurement
 * @param z_star The expected measurement
 *
 * @return The likelihood of the measurement
 */
auto model(auto z, auto z_star, const Parameters& p) {
  // Drawn from discussion with Benjamin of 687D and Probabilistic Robotics by
  // S. Thrun, W. Burgard, and D. Fox (2005).
  // The above book can be found for free at:
  // https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf

  // Desmos graph: https://www.desmos.com/calculator/o7eciudddh

  using namespace _impl;

  /** Probability that the laser hits an unmapped obstacle before the mapped
   * obstacle. */
  auto alpha_short =
      (1 - p.alpha_maxMiss - p.alpha_rand) * exp(-p.lambda_short * z_star);
  /** Probability that the laser hits a mapped obstacle.  */
  auto alpha_hit = (1 - alpha_short - p.alpha_maxMiss - p.alpha_rand);
  /** Probability that the laser hits a mapped obstacle and the measurement
   * is greater than z_max. */
  auto alpha_maxHit =
      1. / 2 * alpha_hit *
      (1 - cwiseErf((p.z_max - z_star) / (p.sigma_hit * sqrt(2))));
  /** Probability that the laser returns a measurement error (z>=z_max). */
  auto alpha_max = alpha_maxHit + p.alpha_maxMiss;

  /**
   * Probability of z, given that the measurement results from hitting a
   * mapped obstacle.
   * Normalized such that ∫_0^infinity p_hit(z)dz = 1, where p_hit(z) does
   * not have bounds.
   */
  auto p_hit = ((0 < z && z < p.z_max)) *
               (2.f * (1 - cwiseErf(-z_star / (p.sigma_hit * sqrt(2)))) *
                normPDF(z_star, p.sigma_hit, z));
  /**
   * Probability of z, given that the measurement results from hitting an
   * unmapped obstacle.
   * Normalized such that ∫_0^z_star p_short(z)dz = 1
   */
  auto p_short = ((0 < z) * (z < z_star))
                     .select(1 / (1 - exp(-p.lambda_short * z_star)) *
                                 p.lambda_short * exp(-p.lambda_short * z),
                             0);
  /**
   * Probability of z, given that the measurement results from a measurement
   * error.
   */
  auto p_max = z >= p.z_max;
  /**
   * Probability of z, given that the measurement results from uniform
   * measurement noise.
   */
  auto p_rand = (0 < z && z < p.z_max) / p.z_max;

  return p_hit * alpha_hit + p_short * alpha_short + p_max * alpha_max +
         p_rand * p.alpha_rand;
};
} // namespace range_model

/**
 * @brief Updates weights with distance sensor measurement.
 */
class Laser {
  public:
    struct Config {
        /** Position relative to the robot. Theta should in standard radians. */
        const lemlib::Pose pose;
        /** Theta of bot in standard radians. Must be constantly updated. */
        const float& botTheta;
        /** Obstacles that sensor might sense. */
        std::vector<lines::Segment>& obstacles;
    };
  protected:
    using Vec2 = Eigen::Vector2f;

    Laser(const Config config);
    // private:
  protected:
    /** Position of sensor relative to center of bot. */
    const Vec2 m_pos;
    /** Angle of sensor relative to bot. Should be in standard radians. */
    const float m_theta;

    /** Holds theta of bot in standard radians. */
    const float& m_botTheta;

    std::vector<lines::Segment>& obstacles;
  public:
    static constexpr float MAX_MEASUREMENT = (2000 * mm).convert(in);
    
    /**
     * @brief Represents a distance measurement from the sensor.
     *
     * Is std::nullopt if sensor is in failure state (disconnected for example).
     * Is >=MAX_MEASUREMENT if sensor failed to measure object.
     * Is <MAX_MEASUREMENT if measurement succeeded.
     */
    typedef std::optional<float> Measurement;

    /**
     * @brief Returns the distance sensed by the sensor and the variance in the
     * reading.
     * @return Measurement of the sensor.
     * @return std::nullopt if sensor is disconnected or malfunctioning.
     * @return >=MAX_MEASUREMENT if object is out of range.
     */
    virtual Measurement sense() = 0;

    /** Creates a ray from this laser object. */
    lines::Ray toRay(const Vec2& robotPos) const;

    /**
     * @brief Implements the PF sensor for the laser sensor.
     * Also prevents the rest of the Laser code from being in the header file.
     *
     * @tparam N Number of particles/weights.
     */
    template <size_t N> struct Sensor
      : public sensors::Sensor<Eigen::Vector2f, N> {
      private:
        static Eigen::Array<float, 2, 1> toEigenArray(Vec2 vec) {
          return vec.array();
        }

        static float minCoeff(Eigen::Array<float, 2, 1> arr) {
          return arr.minCoeff();
        }

        static float getX(Eigen::Array<float, 2, 1> arr) {
          return arr.coeff(0);
        }

        static float getY(Eigen::Array<float, 2, 1> arr) {
          return arr.coeff(1);
        }
      public:
        Laser& laser;

        void update(const Eigen::Array<Vec2, N, 1>& particles,
                    Eigen::Array<float, N, 1>& weights) override {
          auto measurement = laser.sense();
          static range_model::Parameters model_params {
              .alpha_rand = 0.02f,
              .alpha_maxMiss = 0.025f,
              .sigma_hit = 2.f,
              .lambda_short = range_model::density::untrackedObstaclesSkills,
              .z_max = MAX_MEASUREMENT,
          };

          if (!measurement) return;
          auto& z = *measurement;
          const lines::Ray ray = laser.toRay({0, 0});

          constexpr float fieldRadius = dimensions::field::FIELD_LENGTH / 2;

          Eigen::Array<Eigen::Array<float, 2, 1>, N, 1> bothDirDists =
              (((fieldRadius * ray.direction.cwiseSign()) -
                (ray.start + particles))
                   .unaryExpr(&toEigenArray) /
               ray.direction.array());

          auto xDists = bothDirDists.unaryExpr(&getX);
          auto yDists = bothDirDists.unaryExpr(&getY);
          auto expectedDists = xDists.min(yDists);

          weights *= range_model::model(z, expectedDists, model_params);
        }

        Sensor(Laser& laser) : laser(laser) {}
    };
  protected:
    static size_t lastUsedId;
    /**
     * @warning SHOULD ONLY BE USED FOR TELEMETRY PURPOSES!
     * @brief Unique identifier for this sensor.
     * @todo Make a logger class to handle this, and prevent misuse.
     */
    const size_t m_id;
};
}; // namespace sensors