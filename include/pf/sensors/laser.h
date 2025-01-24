#pragma once

#include "Eigen/Core"
#include "Eigen/src/Core/Matrix.h"
#include "lemlib/pose.hpp"
#include "lemlib/util.hpp"
#include "pf/sensors/sensor.h"
#include "pf/util/lines.h"
#include "pf/util/stats.h"
#include <algorithm>
#include "profiler/profiler.h"

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
        Laser& laser;

        void update(const std::array<Vec2, N>& particles,
                    std::array<float, N>& weights) override {
          auto measurement_opt = laser.sense();
          if (!measurement_opt) return;
          auto& measurement = *measurement_opt;
          lines::Ray ray = laser.toRay({0, 0});
          const Vec2 startOffset = ray.start;
          const stats::NormPDF pdf {measurement.dist, measurement.stdDev};

          for (size_t i = 0; i < particles.size(); i++) {
            constexpr float fieldRadius = 72;
            const Vec2 rayStart = startOffset + particles[i];

            const float dist =
                ((fieldRadius * ray.direction.cwiseSign() - rayStart).array() /
                 ray.direction.array())
                    .minCoeff();

            weights[i] *= pdf.eval(dist);
          }
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