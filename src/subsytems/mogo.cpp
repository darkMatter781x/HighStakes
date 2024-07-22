#include "subsystems/mogo.h"
#include "subsystems.h"

MogoClamp::MogoClamp(pros::adi::Pneumatics& pistons)
  : Subsystem(), m_piston(pistons) {}

const MogoClamp::State& MogoClamp::getState() const { return m_state; }

void MogoClamp::update() {}

void MogoClamp::close() {
  m_state = State::CLOSE;
  m_piston.set_value(true);
}

void MogoClamp::open() {
  m_state = State::OPEN;
  m_piston.set_value(false);
}

void MogoClamp::toggle() {
  if (m_state == State::OPEN) this->close();
  else this->open();
}