#pragma once

#include "Eigen/Core"
#include "Eigen/src/Core/Matrix.h"
#include "lemlib/pose.hpp"
#include "pf/sensors/sensor.h"
#include "pf/util/lines.h"
#include "pf/util/stats.h"

namespace sensors {

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

          Eigen::Array<float, N, 1> xDists = bothDirDists.unaryExpr(&getX);
          Eigen::Array<float, N, 1> yDists = bothDirDists.unaryExpr(&getY);
          auto pdfXs = xDists.min(yDists);

          weights *=
              pdfCoeff * Eigen::exp(-.5 * Eigen::pow((pdfXs - mu) / sigma, 2));
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