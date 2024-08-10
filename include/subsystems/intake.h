#pragma once
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "subsystems/mogo.h"

class Intake : public Subsystem {
  public:
    enum State { IN, OUT, IDLE, IN_TO_LIFT, OUT_TO_LIFT };
  private:
    State m_state = State::IDLE;
    pros::MotorGroup& m_motors;
    pros::Optical& m_optical;
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

    Intake(pros::MotorGroup& motors, pros::Optical& optical);
};
