#include "subsystems/intake.h"
#include "pros/llemu.hpp"
#include "pros/optical.hpp"

Intake::Sensor::Sensor(pros::Optical& optical) : m_optical(optical) {
  m_optical.set_led_pwm(100);
}

Intake::Intake(pros::MotorGroup& motors, Sensor& sensor)
  : m_motors(motors), m_sensor(sensor) {}

const Intake::State& Intake::getState() const { return m_state; }

void Intake::update() {
  const State prevState = m_state;

  if (m_sensor.getRing().has_value()) {
    if (m_startSensingRingTimestamp == 0)
      m_startSensingRingTimestamp = pros::millis();
  } else m_startSensingRingTimestamp = 0;

  switch (m_state) {
    case IN: m_motors.move(127); break;
    case OUT: m_motors.move(-127); break;
    case IDLE: m_motors.move(0); break;
    case IN_TO_LIFT:
      m_motors.move(96);
      if (m_startSensingRingTimestamp != 0 &&
          pros::millis() - m_startSensingRingTimestamp > 100)
        setState(OUT_TO_LIFT);
      break;
    case OUT_TO_LIFT:
      m_motors.move(-127);
      if (pros::millis() - m_switchStateTimestamp > 800) setState(IN_TO_LIFT);
      break;
  }

  if (prevState != m_state) update();
}

void Intake::setState(State state) {
  m_switchStateTimestamp = pros::millis();
  m_state = state;
  update();
}

void Intake::stop() { setState(IDLE); }

void Intake::intake() { setState(IN); }

void Intake::outtake() { setState(OUT); }

void Intake::intakeToLift() { setState(IN_TO_LIFT); }