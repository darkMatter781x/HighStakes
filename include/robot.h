#pragma once
#include "config.h"
#include "leds/functions.h"
#include "pf/localization.h"
#include "pros/adi.hpp"
#include "subsystems/selector.h"
#include "lemlib/chassis/chassis.hpp"
#include "subsystems/intake.h"
#include "subsystems/lift.h"
#include "subsystems/ladybrown.h"
#include "color.h"
#include <memory>

/**
 * @brief Provides an abstracted interface for controlling the robot and reading
 * from sensors. Follows the singleton pattern.
 */
class Robot : private lemlib::Chassis {
  private:
    Robot(const RobotConfig& config);

    /** @brief Should ever be one instance of Robot, and that's this one. */
    static Robot instance;
  public:
    const RobotConfig& m_config;
  private:
    Intake m_intake;
    LadyBrown m_lady;
    pros::Controller m_gamepad;
    Intake::Sensor m_intakeSensor;
    Localization m_localization;
    LEDs m_led;

    ALLIANCE m_alliance;
  public:
    /**
     * @brief Gets the robot instance.
     * If it has not been previously constructed (instance == nullptr), then
     * this method will construct it
     */
    inline static Robot& get() { return instance; };

    LadyBrown& lady;
    Intake& intake;
    LEDs& led;

    // pros::adi::Pneumatics& kicker;
    pros::adi::Pneumatics& mogoClamp;
    pros::adi::Pneumatics& doinker;
    pros::adi::Pneumatics& intakeLift;
    pros::Controller& gamepad;
    AutonSelector& selector;
    Localization& localization;

    void setAlliance(ALLIANCE alliance);
    ALLIANCE getAlliance() const;
    COLOR getColor() const;

    using lemlib::Chassis::angularLargeExit;
    using lemlib::Chassis::angularPID;
    using lemlib::Chassis::angularSettings;
    using lemlib::Chassis::angularSmallExit;
    using lemlib::Chassis::arcade;
    using lemlib::Chassis::cancelAllMotions;
    using lemlib::Chassis::cancelMotion;
    using lemlib::Chassis::curvature;
    using lemlib::Chassis::isInMotion;
    using lemlib::Chassis::lateralLargeExit;
    using lemlib::Chassis::lateralPID;
    using lemlib::Chassis::lateralSettings;
    using lemlib::Chassis::lateralSmallExit;
    using lemlib::Chassis::setBrakeMode;
    using lemlib::Chassis::tank;
    using lemlib::Chassis::waitUntil;
    using lemlib::Chassis::waitUntilDone;

    void calibrate();

    void setPose(lemlib::Pose pose, float stdDev = .5, bool radians = false);
    lemlib::Pose getPose(bool radians = false);

    void turnToPoint(lemlib::Pose target, int timeout,
                     lemlib::TurnToPointParams params = {}, bool async = true);
    void turnToHeading(float theta, int timeout,
                       lemlib::TurnToHeadingParams params = {},
                       bool async = true);
    void swingToPoint(lemlib::Pose target, lemlib::DriveSide lockedSide,
                      int timeout, lemlib::SwingToPointParams params = {},
                      bool async = true);
    void swingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout,
                        lemlib::SwingToHeadingParams params = {},
                        bool async = true);
    void moveToPose(lemlib::Pose target, int timeout,
                    lemlib::MoveToPoseParams params = {}, bool async = true);
    void moveToPoint(lemlib::Pose target, int timeout,
                     lemlib::MoveToPointParams params = {}, bool async = true);
};

inline Robot& bot = Robot::get();