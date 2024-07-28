#include "subsystems/lift.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include <cmath>

void Lift::update() {
  if (m_state == State::EMERGENCY_STOP) {
    m_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    m_motors.brake();
    return;
  }
  const float error = calcError();
  const float output = m_pid.update(error);
  bool shouldBrake = m_exitCondition.update(error);

  if (shouldBrake) {
    // if error > smallError, then don't brake and reset exit condition
    if (std::abs(error) > m_config.controllerSettings.smallError)
      m_exitCondition.reset();
    else {
      m_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      m_motors.brake();
      return;
    }
  }

  m_motors.move_voltage(output);
}

Lift::Lift(pros::MotorGroup& motors, pros::Rotation& rotation, Config& config)
  : m_state(State::BOTTOM), m_config(config), m_motors(motors),
    m_rotation(rotation),
    m_pid(m_config.controllerSettings.kP, m_config.controllerSettings.kI,
          m_config.controllerSettings.kD,
          m_config.controllerSettings.windupRange, true),
    m_exitCondition(m_config.controllerSettings.smallError,
                    m_config.controllerSettings.smallErrorTimeout) {
  // set the rotation sensor position to its absolute angle
  rotation.set_position(rotation.get_angle());
}

float Lift::getTargetAngle() const {
  switch (m_state) {
    case State::BOTTOM: return m_config.bottom;
    case State::MIDDLE: return m_config.middle;
    case State::TOP: return m_config.top;
    default: return NAN;
  }
}

float Lift::calcLiftAngle() const {
  return m_rotation.get_position() * m_config.gearRatio;
}

float Lift::calcError() const { return getTargetAngle() - calcLiftAngle(); }

const Lift::State& Lift::getState() { return m_state; }

// state setters
void Lift::setState(State state) {
  m_state = state;
  m_exitCondition.reset();
}

void Lift::emergencyStop() { setState(State::EMERGENCY_STOP); }

void Lift::goToBottom() { setState(State::BOTTOM); }

void Lift::goToMiddle() { setState(State::MIDDLE); }

void Lift::goToTop() { setState(State::TOP); }

void Lift::goUp() {
  switch (m_state) {
    case State::BOTTOM: goToMiddle(); break;
    case State::MIDDLE: goToTop(); break;
    default: break;
  }
}

void Lift::goDown() {
  switch (m_state) {
    case State::MIDDLE: goToBottom(); break;
    case State::TOP: goToMiddle(); break;
    default: break;
  }
}