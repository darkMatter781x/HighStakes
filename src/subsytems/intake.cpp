#include "subsystems/intake.h"

Intake::Intake(pros::MotorGroup& motors, const MogoClamp::State& mogoState)
  : m_motors(motors), m_mogoState(mogoState) {}

const Intake::State& Intake::getState() const { return m_state; }

void Intake::update() {
  if (m_mogoState == MogoClamp::State::OPEN) m_state = State::IDLE;
  switch (m_state) {
    case IN: m_motors.move(127); break;
    case OUT: m_motors.move(-127); break;
    case IDLE: m_motors.move(0); break;
  }
}

void Intake::stop() { m_state = State::IDLE; }

void Intake::intake() { m_state = State::IN; }

void Intake::outtake() { m_state = State::OUT; }