#pragma once
#include "pros/motor_group.hpp"
#include "subsystems/mogo.h"

class Intake : public Subsystem {
  public:
    enum State { IN, OUT, IDLE };
  private:
    State m_state = State::IDLE;
    pros::MotorGroup& m_motors;
    const MogoClamp::State& m_mogoState;
  public:
    void stop();
    void intake();
    void outtake();

    void update() override;

    const State& getState() const;

    Intake(pros::MotorGroup& motors, const MogoClamp::State& mogoState);
};
