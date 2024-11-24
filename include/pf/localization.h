#pragma once

#include "Eigen/Core"
#include "lemlibOdom.h"
#include "pf.h"
#include "resamplers/systematic.h"
#include "pros/distance.hpp"
#include "pf/util/lines.h"
#include "subsystems.h"

/**
 * @brief Implementation of particle filter. Updates LemLib's Odom so that
 * lemlib::Chassis should work.
 */
class Localization : Subsystem {
  public:
    /** Config for constructing distance sensors*/
    struct DistSensorConfig {
        /** Position of sensor relative to center of bot. */
        const lemlib::Pose pose;
        /** V5 distance sensor object. */
        pros::Distance& sensor;
    };

    /** Config for constructing localizer */
    struct Config {
        lemlib::OdomSensors odomSensors;
        lemlib::Drivetrain drivetrain;
        /** Obstacles that distance sensors might sense. */
        std::vector<lines::Segment> obstacles;
        std::vector<DistSensorConfig> distances;

        /**
         * @brief Represents noise in system model.
         * @symbol Q
         */
        Eigen::Matrix2f covar;
    };
  private:
    // Number of particles
    static constexpr size_t N = 100;
    /** Particle type */
    typedef Eigen::Vector2f P;
    /** Particle filter type */
    typedef ParticleFilter<P, N> PF;

    PF m_pf;
    LemLibOdom m_odom;

    /** Used for displacement calculation */
    lemlib::Pose prevPose;

    /** @brief Moves particle based on lemlib odom readings */
    struct OdomPredictor : PF::Predictor {
        /**
         * @brief Represents noise in system model.
         * @symbol Q
         */
        Eigen::Matrix2f m_covar;
        /** @brief Represents change in position, according to LemLib odom. */
        P m_displacement;
        /** @brief Updates particles according to LemLib odom. */
        void predict(std::array<P, N>& particles) override;

        using RandomFunc =
            std::function<Eigen::Vector2f(Eigen::Matrix2f& covar)>;
        /** Used to introduce noise into the system. Must be gaussian and
         * centered around a mean of [0,0]. */
        RandomFunc randomFunc;

        OdomPredictor(Eigen::Matrix2f covar, RandomFunc randomFunc);
    } m_predictor;

    std::vector<lines::Segment> m_obstacles;
    std::vector<std::unique_ptr<PF::Sensor>> m_sensors;

    resamplers::Systematic<N> m_resampler;
  public:
    void update() override;

    /**
     * @brief Set the pose of the robot. Theta must be in standard radians.
     *
     * @param pose New pose of the robot. Theta must be in standard radians.
     * @param stdDev How much to trust the new pose
     * @param radians If theta is in radians
     * @param standardPos If the pose is in standard position (eg. 0 theta
     * corresponds to +x axis)
     */
    void setPose(lemlib::Pose pose, float stdDev, bool radians = false,
                 bool standardPos = false);

    /**
     * @brief Construct a new Localizer object
     *
     * @param config Configuration for the localizer
     */
    Localization(const Config& config);
};