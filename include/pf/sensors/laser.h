#pragma once

#include "Eigen/Core"

#include "Eigen/src/Core/Array.h"
#include "Eigen/src/Core/Matrix.h"
#include "lemlib/pose.hpp"
#include "pf/sensors/sensor.h"
#include "pf/util/lines.h"
#include "pf/util/stats.h"
#include "units/units.hpp"
#include "math.h"

namespace sensors {
// constexpr double DENSITY_OF_UNTRACKED_OBSTACLES =

auto square(auto a) { return a * a; }

constexpr double getDensityOfUntrackedObstacles() {
  Area ring = square(7 * in) * M_PI;
  Area mogo = ring;
  Area robot = square(18 * in);
  Area ladderPost = 2 * in * 2 * in;
  Area field = square(144 * in);
  Area ringStack = ring;
  Area untrackedObstacles =
      mogo * 5 + robot * 3 + ladderPost * 4 + ringStack * 4;

  Area totalNotIncludingMe = field - robot;
  Number density = (untrackedObstacles / totalNotIncludingMe);
  return density.internal();
}

auto my_erf(auto x) {
  return x.unaryExpr([](auto y) { return erf(y); });
}

auto normPDF(auto mu, auto sigma, auto x) {
  return 1 / (sqrt(2 * M_PI) * sigma) *
         exp(-1. / 2 * square(mu - x) / square(sigma));
};

struct RangeSensorModel {
    struct Parameters {
        float alpha_hit;
        float alpha_short;
        float alpha_max;
        float alpha_rand;
        float sigma_hit;
        float lambda_short;
        float z_max;
    } p;

    /**
     * @brief
     *
     * @param z The actual measurement
     * @param z_star The expected measurement
     * @return T The likelihood of the measurement
     */
    auto prob(auto z, auto z_star) {
      auto p_hit =
          ((0 < z && z < p.z_max)) *
          (sqrt(M_PI) / 2.f * abs(my_erf(-z_star) - my_erf(p.z_max - z_star)) *
           normPDF(z_star, p.sigma_hit, z));
      auto p_short = ((0 < z) * (z < z_star))
                         .select(1 / (1 - exp(-p.lambda_short * z_star)) *
                                     p.lambda_short * exp(-p.lambda_short * z),
                                 0); // 0 < z < z_star
      // auto p_max = z >= p.z_max; // z >= z_max
      // auto p_rand = (0 < z && z < p.z_max) / p.z_max; // 0 < z < z_max
      return p_hit * p.alpha_hit + p_short * p.alpha_short;
      //  p_max * p.alpha_max + p_rand * p.alpha_rand;
    };
};

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
    struct Measurement {
        float dist;
        float stdDev;
    };

    /**
     * @brief Returns the distance sensed by the sensor and the variance in the
     * reading.
     * @return Measurement of the sensor.
     */
    virtual std::optional<Measurement> sense() = 0;

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
          auto measurement_opt = laser.sense();
          static RangeSensorModel model {
              .p = {
                  .alpha_hit = 0.9,
                  .alpha_short = 0.1,
                  .alpha_max = 0,
                  .alpha_rand = 0,
                  .sigma_hit = 8.f,
                  .lambda_short = (float)getDensityOfUntrackedObstacles(),
                  .z_max = 2000.f,
              }};

          if (!measurement_opt) return;
          auto& measurement = *measurement_opt;
          const lines::Ray ray = laser.toRay({0, 0});

          constexpr float fieldRadius = 140.25 / 2;
          const float mu = measurement.dist;
          const float sigma = measurement.stdDev;
          const float pdfCoeff = 1 / (sqrt(2 * M_PI) * sigma);

          Eigen::Array<Eigen::Array<float, 2, 1>, N, 1> bothDirDists =
              (((fieldRadius * ray.direction.cwiseSign()) -
                (ray.start + particles))
                   .unaryExpr(&toEigenArray) /
               ray.direction.array());

          auto xDists = bothDirDists.unaryExpr(&getX);
          auto yDists = bothDirDists.unaryExpr(&getY);
          auto expectedDists = xDists.min(yDists);

          weights *= model.prob(mu, expectedDists);
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