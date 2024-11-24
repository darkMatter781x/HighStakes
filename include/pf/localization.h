#pragma once

#include "Eigen/Core"
#include "lemlibOdom.h"
#include "pf.h"
#include "pros/distance.hpp"
#include "resamplers/systematic.h"
#include "subsystems.h"
#include <vector>

/**
 * @brief Implementation of particle filter. Updates LemLib's Odom so that
 * lemlib::Chassis should work.
 */
class Localization : Subsystem {
  public:
    /** Represent a distance sensor */
    struct DistanceSensor {
        /** Pros sensor object */
        pros::Distance& sensor;
        /** Relative pose to robot. Theta should be in standard radians. */
        const lemlib::Pose pose;
    };

    /** Config for constructing localizer */
    struct Config {
        lemlib::OdomSensors& odomSensors;
        lemlib::Drivetrain& drivetrain;
        std::vector<DistanceSensor> distances;
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

    std::vector<DistanceSensor> m_dists;

    /** @brief Moves particle based on lemlib odom readings */
    struct OdomPredictor : PF::Predictor {
        /**
         * @brief Represents noise in system model.
         * @sybmol Q
         */
        Eigen::Matrix2f m_cov;
        /** @brief Represents change in position, according to LemLib odom. */
        P m_displacement;
        /** @brief Updates particles according to LemLib odom. */
        void predict(std::array<P, N>& particles) override;
    } m_predictor;

    struct DistSensorUpdater : PF::Updater {
        const std::vector<DistanceSensor>& m_dists;

        DistSensorUpdater(const std::vector<DistanceSensor>& dists);

        void update(const std::array<P, N>& particles,
                    std::array<float, N>& weights) override;
    } m_updater;

    resamplers::Systematic<N> m_resampler;
  public:
    void update() override;

    /**
     * @brief Set the Pose object
     *
     * @param pose New pose of the robot
     * @param stdDev How much to trust the new pose
     */
    void setPose(lemlib::Pose pose, float stdDev);

    /**
     * @brief Construct a new Localizer object
     *
     * @param config Configuration for the localizer
     */
    Localization(const Config& config);
};