#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include <cstdio>

/**
 * @brief Configuration for the robot. Provides all devices and dimensions of
 * the robot
 */
struct RobotConfig {
    struct Motors {
        pros::MotorGroup left;
        pros::MotorGroup right;
        pros::MotorGroup intake;
        pros::MotorGroup lift;
      private:
        friend struct RobotConfig;
        static Motors motors;
    };

    struct Pneumatics {
        pros::adi::Pneumatics mogoClamp;
        pros::adi::Pneumatics ringClaw;
      private:
        friend struct RobotConfig;
        static Pneumatics pneumatics;
    };

    struct Sensors {
        pros::Rotation vert;
        pros::Rotation hori;
        pros::Rotation lift;
        pros::IMU imu;
      private:
        friend struct RobotConfig;
        static Sensors sensors;
    };

    struct Dimensions {
        const float trackWidth;
        const float driveWheelDiameter;
        const float driveWheelRpm;
        const float driveEncGearRatio;

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
        pros::adi::LED lift;
        pros::adi::LED leftUnderGlow;
        pros::adi::LED rightUnderGlow;
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
