#pragma once
#include "config.h"
#include "subsystems/intake.h"
#include "subsystems/lift.h"

/**
 * @brief Provides an abstracted interface for controlling the robot and reading
 * from sensors. Follows the singleton pattern.
 */
class Robot : public lemlib::Chassis {
  private:
    Robot(const RobotConfig& config);

    /** @brief Should ever be one instance of Robot, and that's this one. */
    static Robot instance;

    const RobotConfig& m_config;

    MogoClamp m_mogo;
    Intake m_intake;
    Lift m_lift;
  public:
    /**
     * @brief Gets the robot instance.
     * If it has not been previously constructed (instance == nullptr), then
     * this method will construct it
     */
    inline static Robot& get() { return instance; };

    Lift& lift;
    Intake& intake;
    MogoClamp& mogo;
};

inline Robot& bot = Robot::get();