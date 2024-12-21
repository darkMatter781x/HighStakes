#pragma once

#include "Eigen/Core"
#include "Eigen/src/Core/Matrix.h"
#include "pf.h"
#include "pf/odom/odom.h"
#include "pros/rtos.hpp"
#include "resamplers/systematic.h"
#include "pf/util/lines.h"
#include "subsystems.h"
#include <memory>

/**
 * @brief Implementation of particle filter. Updates LemLib's Odom so that
 * lemlib::Chassis should work.
 */
class Localization : Subsystem {
  private:
    pros::Mutex m_mutex;
  public:
    // Number of particles
    static constexpr size_t N = 500;
    /** Particle type */
    typedef Eigen::Vector2f P;
    /** Particle filter type */
    typedef ParticleFilter<P, N> PF;

    typedef std::vector<std::unique_ptr<PF::Sensor>> SensorVector;

    /** Config for constructing sensors, given to sensor factory */
    struct SensorFactoryConfig {
        /** V5 distance sensor object. */
        const lemlib::Pose& botPose;

        /** Obstacles that a laser sensor might sense. */
        std::vector<lines::Segment>& obstacles;
    };

    /** Config for constructing localizer */
    struct Config {
        std::shared_ptr<Odom> odom;
        /** Obstacles that distance sensors might sense. */
        std::vector<lines::Segment> obstacles;
        std::function<SensorVector(SensorFactoryConfig)> sensorFactory;

        /**
         * @brief Represents noise in system model.
         * @symbol Q
         */
        Eigen::Matrix2f covar;
    };
  private:
    PF m_pf;
    std::shared_ptr<Odom> m_odom;

    /** Used for displacement calculation */
    lemlib::Pose m_prevPose;

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

        OdomPredictor(Eigen::Matrix2f covar);
    } m_predictor;

    std::vector<lines::Segment> m_obstacles;
    SensorVector m_sensors;

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
     * @brief Calibrates the odometry sensors, and enables the particle filter.
     */
    void calibrate();

    /**
     * @brief Construct a new Localizer object
     *
     * @param config Configuration for the localizer
     */
    Localization(const Config& config);
};