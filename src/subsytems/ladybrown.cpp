#include "subsystems/ladybrown.h"
#include "lemlib/util.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "units/units.hpp"
#include "units/Angle.hpp"

using units::sin;

/**
 * @param angle Angle in degrees
 * @return angle between 0 and 360.
 */
constexpr float sanitizeAngle(float angle) {
  return std::fmod(std::fmod(angle, 360) + 360, 360);
}

template <typename T> T clamp(T val, T min, T max) {
  if (val < min) return min;
  if (val > max) return max;
  return val;
}

/** Constant for opposing torque from force of gravity */
constexpr Voltage K_antiG = .75 * volt;

/** Angle that shall not be crossed in degrees */
constexpr float doNotCrossAngle = 45;
constexpr Voltage maxPower = 8720 * mvolt;

void LadyBrown::update() {
  // Handle non-feedback control modes
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

  // PID CONTROL

  /** Angle of the Lady Brown */
  const Angle thetaLB = calcLBAngle() * deg;
  /** Angle of the force of gravity */
  const Angle thetaF_g = 90 * deg - thetaLB - 180 * deg;
  const Voltage antiGPower = sin(thetaF_g) * K_antiG;

  const float error = calcError();
  const Voltage pidPower = m_pid.update(error) * mvolt;

  const Voltage desiredPower = antiGPower + pidPower;
  const Voltage limitedPower = clamp(desiredPower, -maxPower, maxPower);

  m_motors.move_voltage(limitedPower.convert(mvolt));

  // Clear screen
  // printf("\033[2J");
  // printf("calcLBAngle():\t%3.2fdeg\n", calcLBAngle());
  // printf("getTargetAngle():\t%3.2fdeg\n", getTargetAngle());
  // printf("error:\t%3.2fdeg\n", error);
  // printf("limitedPower:\t%2.3fV\n", limitedPower.convert(volt));

  // static size_t startTime = pros::millis();
  // // Initialize rotation sensor with proper position
  // if (pros::millis() - startTime < 100) {
  //   // If rotation is something like 350, then set it to 10
  //   int currCentidegree = m_rotation.get_position();
  //   printf("LadyBrown::update(): currCentidegree: %i\n", currCentidegree);
  //   if (currCentidegree > 180 * 100) {
  //     printf("setting position\n");
  //     m_rotation.set_position((currCentidegree - (360 * 100)));
  //   }
  // }

  // const float error = calcError();
  // const float output = std::min(8720.0f, m_pid.update(error));
  // bool shouldBrake = m_exitCondition.update(error);
  // if (shouldBrake) {
  //   // if error > smallError, then don't brake and reset exit condition
  //   if (std::abs(error) > m_config.controllerSettings.smallError)
  //     m_exitCondition.reset();
  //   else {
  //     switch (m_state) {
  //       case State::IDLE:
  //         m_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  //         break;
  //       default: m_motors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); break;
  //     };
  //     m_motors.brake();
  //     if (bot.selector.isLocked()) pros::lcd::print(2, "braking");
  //     return;
  //   }
  // }
  // if (bot.selector.isLocked()) pros::lcd::print(2, "output: %4.2f", output);
  // m_motors.move_voltage(output);

  // if (m_state == LadyBrown::IDLE && calcLBAngle() > 350 && calcLBAngle() <
  // 30) {
  //   m_rotation.set_position(0);
  // }

  /*if (m_state == LadyBrown::SCORE) {
    if (calcLBAngle() > 160) {
      dir = AngDir::CCW_COUNTERCLOCKWISE;
    } else {
      dir = AngDir::AUTO;
    }
  }
  if (m_state == LadyBrown::IDLE) {
    if (calcLBAngle() < 290 && calcLBAngle() > 75) {
      dir = AngDir::CW_CLOCKWISE;
    } else {
      dir = AngDir::AUTO;
    }
  } */
}

LadyBrown::LadyBrown(pros::MotorGroup& motors, pros::Rotation& rotation,
                     Config& config)
  : m_state(State::IDLE), m_config(config), m_motors(motors),
    m_rotation(rotation),
    m_pid(m_config.controllerSettings.kP, m_config.controllerSettings.kI,
          m_config.controllerSettings.kD,
          m_config.controllerSettings.windupRange, false),
    m_exitCondition(m_config.controllerSettings.smallError,
                    m_config.controllerSettings.smallErrorTimeout) {}

float LadyBrown::getTargetAngle() const {
  switch (m_state) {
    case State::IDLE: return m_config.idle;
    case State::COLLECT: return m_config.collect;
    case State::IDLE_SCORE: return m_config.idle_score;
    case State::SCORE: return m_config.score;
    case State::RUN_INTO_ALLY: return m_config.run_into_ally;
    case State::FLIP: return m_config.flip;
    default: return NAN;
  }
}

float LadyBrown::calcLBAngle() const {
  // Prevent lift from moving if rotation sensor is not installed
  if (!m_rotation.is_installed()) return getTargetAngle();
  return m_rotation.get_position() / 100.0;
}

float LadyBrown::calcError() const {
  const float currAngle = sanitizeAngle(calcLBAngle());
  const float targetAngle = sanitizeAngle(getTargetAngle());

  const float shortError =
      lemlib::angleError(targetAngle, currAngle, false, AngDir::AUTO);
  const float longError =
      -lemlib::sgn(shortError) * (360 - std::abs(shortError));

  /** Error to angle that we dont want to go to */
  const float doNotCrossError =
      lemlib::angleError(doNotCrossAngle, currAngle, false);

  // If short direction is the same as the direction to the do not cross zone
  if (lemlib::sgn(shortError) == lemlib::sgn(doNotCrossError)) {
    // If target is before do not cross zone
    if (std::abs(shortError) < std::abs(doNotCrossError)) return shortError;
    else return longError;
  } else return shortError;
}

const LadyBrown::State& LadyBrown::getState() { return m_state; }

void LadyBrown::setState(State state) {
  m_state = state;
  m_exitCondition.reset();
}

void LadyBrown::emergencyStop() { setState(State::EMERGENCY_STOP); }

void LadyBrown::goToIdle() { setState(State::IDLE); }

void LadyBrown::collect() { setState(State::COLLECT); }

void LadyBrown::goToIdleScore() { setState(State::IDLE_SCORE); }

void LadyBrown::score() { setState(State::SCORE); }

void LadyBrown::runIntoAlly() { setState(State::RUN_INTO_ALLY); }

void LadyBrown::flip() { setState(State::FLIP); }
