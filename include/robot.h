#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"

#pragma once

/**
 * @brief Configuration for the robot. Provides all devices and dimensions of
 * the robot
 */
struct RobotConfig {
    struct Motors {
        pros::MotorGroup& left;
        pros::MotorGroup& right;
        pros::MotorGroup& intake;
        pros::MotorGroup& lift;
    };

    struct Pneumatics {
        pros::adi::Pneumatics& mogoClamp;
        pros::adi::Pneumatics& ringClaw;
    };

    struct Sensors {
        pros::Rotation& vert;
        pros::Rotation& hori;
        pros::IMU& imu;
    };

    struct Dimensions {
        float trackWidth;
        float driveWheelDiameter;
        float driveWheelRpm;
        float driveEncGearRatio;

        float vertEncDiameter;
        float vertEncDistance;
        float vertEncGearRatio;

        float horiEncDiameter;
        float horiEncDistance;
        float horiEncGearRatio;

        float drivetrainWidth;
        float drivetrainLength;
    };

    struct LEDs {
        pros::adi::LED lift;
        pros::adi::LED underGlow;
    };

    struct Tunables {
        const lemlib::ControllerSettings& lateralController;
        const lemlib::ControllerSettings& angularController;
        const float horizontalDrift;
        const float imuGain;
    };

    Motors& motors;
    Pneumatics& pneumatics;
    Sensors& sensors;
    const Dimensions& dimensions;
    const Tunables& tunables;
    LEDs& leds;
};

/**
 * @brief Provides an abstracted interface for controlling the robot and reading
 * from sensors. Follows the singleton pattern.
 */
class Robot : lemlib::Chassis {
  private:
    /** @brief Should ever be one instance of Robot, and that's this one. */
    static Robot* instance;

    Robot(RobotConfig& config);
        
    const RobotConfig& m_config;
  public:
    /**
     * @brief Gets the robot instance.
     * If it has not been previously constructed (instance == nullptr), then
     * this method will construct it
     */
    static Robot* get();

    /**
     * @brief the config used to construct the single instance of the robot
     */
    static const RobotConfig& config;
};