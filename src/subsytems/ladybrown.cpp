#include "subsystems/ladybrown.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "robot.h"

void LadyBrown::update() {
  switch (m_state) {
    case State::EMERGENCY_STOP: {
      m_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      m_motors.brake();
      return;
    }
    case State::COAST: {
      m_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      m_motors.brake();
      return;
    }
    default: break;
  }
  const float error = calcError();
  const float output = std::min(8720.0f, m_pid.update(error));
  bool shouldBrake = m_exitCondition.update(error);
  if (shouldBrake) {
    // if error > smallError, then don't brake and reset exit condition
    if (std::abs(error) > m_config.controllerSettings.smallError)
      m_exitCondition.reset();
    else {
      switch (m_state) {
        case State::IDLE:
          m_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
          break;
        default: m_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); break;
      };
      m_motors.brake();
      if (bot.selector.isLocked()) pros::lcd::print(2, "braking");
      return;
    }
  }
  if (bot.selector.isLocked()) pros::lcd::print(2, "output: %4.2f", output);
  m_motors.move_voltage(output);
}

LadyBrown::LadyBrown(pros::MotorGroup& motors, pros::Rotation& rotation,
                     Config& config)
  : m_state(State::IDLE), m_config(config), m_motors(motors),
    m_rotation(rotation),
    m_pid(m_config.controllerSettings.kP, m_config.controllerSettings.kI,
          m_config.controllerSettings.kD,
          m_config.controllerSettings.windupRange, true),
    m_exitCondition(m_config.controllerSettings.smallError,
                    m_config.controllerSettings.smallErrorTimeout) {}

float LadyBrown::getTargetAngle() const {
  switch (m_state) {
    case State::IDLE: return m_config.idle;
    case State::COLLECT: return m_config.collect;
    case State::SCORE: return m_config.score;
    default: return NAN;
  }
}

float LadyBrown::calcLBAngle() const {
  // Prevent lift from moving if rotation sensor is not installed
  if (!m_rotation.is_installed()) return getTargetAngle();
  return m_rotation.get_position() / 100.0;
}

float LadyBrown::calcError() const {
  using AngDir = lemlib::AngularDirection;
  // constexpr AngDir CCW = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE;

  const float currAngle = calcLBAngle();
  // // If past max angle, then only turn counter clockwise (towards idle)
  // // This prevents the lady brown from turning through itself.
  // const AngDir dir = m_config.score < currAngle ? CCW : AngDir::AUTO;
  AngDir dir = AngDir::AUTO;
  if (m_state == LadyBrown::IDLE){
    dir = AngDir::CW_CLOCKWISE;
  } else {
    dir = AngDir::CCW_COUNTERCLOCKWISE;
  }
  return lemlib::angleError(getTargetAngle(), currAngle, false, dir);
}

const LadyBrown::State& LadyBrown::getState() { return m_state; }

void LadyBrown::setState(State state) {
  m_state = state;
  m_exitCondition.reset();
}

void LadyBrown::emergencyStop() { setState(State::EMERGENCY_STOP); }

void LadyBrown::goToIDLE() { setState(State::IDLE); }

void LadyBrown::goToCOLLECT() { setState(State::COLLECT); }

void LadyBrown::goToSCORE() { setState(State::SCORE); }

void LadyBrown::nextState() {
  switch (m_state) {
    case State::IDLE: goToCOLLECT(); break;
    case State::COLLECT: goToSCORE(); break;
    case State::SCORE: goToIDLE(); break;
    default: goToIDLE(); break;
  }
}
