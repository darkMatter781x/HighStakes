#pragma once
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "subsystems/mogo.h"
#include "alliance.h"
#include <optional>

class Intake : public Subsystem {
  public:
    /** @brief Wrapper for the intake's optical sensor. */
    class Sensor {
      private:
        pros::Optical& m_optical;
      public:
        Sensor(pros::Optical& optical);
        /** @brief Returns the color of the ring if it is present. */
        std::optional<COLOR> getRing() const;
    };

    enum State { IN, OUT, IDLE, IN_TO_LIFT, OUT_TO_LIFT };
  private:
    State m_state = State::IDLE;
    pros::MotorGroup& m_motors;
    Sensor& m_sensor;
    uint32_t m_switchStateTimestamp = pros::millis();
    /** if = 0, then we were not sensing the ring */
    uint32_t m_startSensingRingTimestamp = 0;
  public:
    void setState(State state);
    void stop();
    void intake();
    void outtake();
    void intakeToLift();

    void update() override;

    const State& getState() const;

    Intake(pros::MotorGroup& motors, Sensor& sensor);
};
