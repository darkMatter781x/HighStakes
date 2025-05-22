#pragma once

#include "leds/strip.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/optical.hpp"
#include <vector>

/**
 * @brief Configuration for the robot. Provides all devices and dimensions of
 * the robot
 */
struct RobotConfig {
    struct Motors {
        pros::MotorGroup left;
        pros::MotorGroup right;
        pros::MotorGroup intake;
        pros::MotorGroup LadyBrown;
      private:
        friend struct RobotConfig;
        static Motors motors;
    };

    struct Pneumatics {
        pros::adi::Pneumatics mogoClamp;
        pros::adi::Pneumatics kicker;
        pros::adi::Pneumatics leftDoinker;
        pros::adi::Pneumatics rightDoinker;
        pros::adi::Pneumatics intakeLift;
      private:
        friend struct RobotConfig;
        static Pneumatics pneumatics;
    };

    struct Sensors {
        pros::Rotation vert;
        pros::Rotation hori;
        pros::Rotation lb;
        pros::IMU imu;
        pros::Optical intake;
        pros::adi::AnalogIn intakeKick;
        pros::adi::Potentiometer autonSelector;

        /** Config for constructing distance sensors*/
        struct DistSensorConfig {
            /** Position of sensor relative to center of bot. Theta should be in
             * standard radians. */
            const lemlib::Pose pose;
            /** V5 distance sensor object. */
            pros::Distance& sensor;
        };

        std::vector<DistSensorConfig> lasers;
      private:
        friend struct RobotConfig;
        static Sensors sensors;
    };

    struct Dimensions {
        const float trackWidth;
        const float driveWheelDiameter;
        const float driveWheelRpm;

        const float vertEncDiameter;
        const float vertEncDistance;
        const float vertEncGearRatio;

        const float horiEncDiameter;
        const float horiEncDistance;
        const float horiEncGearRatio;

        const float drivetrainWidth;
        const float drivetrainLength;

        static Dimensions dimensions;
    };

    struct Tunables {
        const lemlib::ControllerSettings& lateralController;
        const lemlib::ControllerSettings& angularController;
        const float horizontalDrift;
        const float imuGain;

        lemlib::ExpoDriveCurve driveCurve;
      private:
        friend struct RobotConfig;
        static Tunables tunables;
    };

    struct LEDs {
        std::shared_ptr<LedStrip> leftLift;
        std::shared_ptr<LedStrip> rightLift;
        std::shared_ptr<LedStrip> leftUnderGlow;
        std::shared_ptr<LedStrip> rightUnderGlow;
        // private:
        friend struct RobotConfig;
        static LEDs leds;
    };

    Motors& motors;
    Pneumatics& pneumatics;
    Sensors& sensors;
    LEDs& leds;
    Dimensions& dimensions;
    Tunables& tunables;

    lemlib::OdomSensors makeSensors() const;
    lemlib::Drivetrain makeDrivetrain() const;
  private:
    friend class Robot;
    static const RobotConfig config;
};
