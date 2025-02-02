#include "subsystems/intake.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems/ladybrown.h"
#include "robot.h"
#include <cmath>
#include <cstdio>

#define POWER 127

// TODO: use mutex for state changes

std::string destToStr(Intake::DESTINATION dest) {
  switch (dest) {
    case Intake::MOGO: return "MOGO";
    case Intake::KICK: return "KICK";
    case Intake::LB: return "LIFT";
  }
}

void printFiltering(Intake::Filtering& filter) {
   printf("filtering:\tr:\t%s\tb:\t%s\n", destToStr(filter.red).c_str(),
          destToStr(filter.blue).c_str());
}

Intake::Intake(pros::MotorGroup& motors, Sensor& sensor,
               pros::adi::Pneumatics& kicker, Config conf)
  : m_motors(motors), m_sensor(sensor), m_kicker(kicker),
    m_visitor([this](BaseState& state) { state.update(*this); }), m_conf(conf) {
  m_motors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

const Intake::State& Intake::getState() const { return m_state; }
//std::optional<COLOR> Intake::getFilterColor() { return  }

// const Intake::DESTINATION& Intake::getDestination() const { return dest; }

void Intake::BaseState::start(
    Intake& intake) { /* printf("starting state\n"); */ }

void Intake::BaseState::end(Intake& intake) { /* printf("ending state\n"); */ }

Intake::FilteringState::FilteringState(Filtering afterState, COLOR color)
  : color(color), afterState(afterState) {}

void Intake::FilteringState::switchDest(Intake& intake, DESTINATION dest) {
  afterState.setDest(color, dest);
}

float Intake::FilteringState::getInchesTraveled(const Intake& intake) const {
  // printf("getting inches traveled, pos: %4.2f\n",
  //        intake.m_motors.get_position() - startPosition);
  return (intake.m_motors.get_position() - startPosition) /* rots */ /
         (intake.m_conf.chainRatio /* rots per inch */);
}

void Intake::FilteringState::start(Intake& intake) {
  Intake::BaseState::start(intake);
   printf("starting filtering state\n");
  startTime = pros::millis();
  startPosition = intake.m_motors.get_position();
  // intake.m_optical.set_led_pwm(100);
}

void Intake::FilteringState::end(Intake& intake) {
  Intake::BaseState::end(intake);
  // printf("ending filtering state\n");
  // intake.m_optical.set_led_pwm(0);
}

Intake::IntakingToKick::IntakingToKick(Filtering afterState, COLOR color)
  : FilteringState(afterState, color) {}

Intake::KickingRing::KickingRing(Filtering afterState, COLOR color)
  : FilteringState(afterState, color) {}

Intake::IntakingToLB::IntakingToLB(Filtering afterState, COLOR color)
  : FilteringState(afterState, color) {}

Intake::IntakingToMogo::IntakingToMogo(Filtering afterState, COLOR color)
  : FilteringState(afterState, color) {}

void Intake::IntakingToKick::update(Intake& intake) {
  intake.m_motors.move(POWER);
   if (pros::millis() % 200 < 10) printf("intaking to kick\n");
  printFiltering(afterState);
  if (getInchesTraveled(intake) > intake.m_conf.intakingToKickDuration) {
     printf("naturally switching to idle\n");
    intake.setState(KickingRing {afterState, color});
  }
}

void Intake::IntakingToKick::switchDest(Intake& intake, DESTINATION dest) {
  FilteringState::switchDest(intake, dest);
  printf("switch from intaking to kick to %s\n", destToStr(dest).c_str());
  printFiltering(afterState);
  if (dest == LB) {
    IntakingToLB newState(afterState, color);
    newState.startTime = this->startTime;
    intake.setState(newState);
  } else if (dest == MOGO) {
    IntakingToMogo newState(afterState, color);
    newState.startTime = this->startTime;
    intake.setState(newState);
  }
}

void Intake::KickingRing::update(Intake& intake) {
  intake.m_motors.move(-127);
  printf("idling to kick\n");
  printFiltering(afterState);
  if (pros::millis() % 200 < 10) printf("idling to kick\n");
  if (pros::millis() - startTime > intake.m_conf.intakingToKickDuration) {
     printf("naturally switching to filtering\n");
    intake.setState(afterState);
    intake.numOfKickRings++;
    intake.numOfRings++;
  }
}

void Intake::KickingRing::start(Intake& intake) {
  printf("idling to kick\n");
  Intake::FilteringState::start(intake);
  // intake.m_kicker.set_value(true);
}

void Intake::KickingRing::end(Intake& intake) {
  printf("idling to kick\n");
  Intake::FilteringState::start(intake);

  // intake.m_kicker.set_value(false);
}

void Intake::IntakingToLB::update(Intake& intake) {
  intake.m_motors.move(POWER);
  // if (pros::millis() % 200 < 10) printf("intaking to lift\n");
  printf("intaking to lift, %4.2f\n", getInchesTraveled(intake));
}

/*void Intake::OuttakingToLift::update(Intake& intake) {
  // If we quickly change the direction of the intake,
  // it will decrease consistency of the intaking to the lift.
  // So, we delay the change a bit.
  if (pros::millis() - startTime < 200) {
    intake.m_motors.move(0);
  } else intake.m_motors.move(-96);

  // if (pros::millis() % 200 < 10) printf("outtaking to lift\n");
  if (pros::millis() - startTime >
      intake.m_conf.outtakingToLiftDuration + 200) {
    intake.setState(afterState);
    intake.numOfLiftRings++;
    intake.numOfRings++;
  }
}

*/

void Intake::IntakingToLB::switchDest(Intake& intake, DESTINATION dest) {
  FilteringState::switchDest(intake, dest);
  if (dest == KICK) {
    IntakingToKick newState(afterState, color);
    newState.startTime = this->startTime;
    intake.setState(newState);
  } else if (dest == MOGO) {
    IntakingToMogo newState(afterState, color);
    newState.startTime = this->startTime;
    intake.setState(newState);
  }
}

void Intake::IntakingToMogo::update(Intake& intake) {
  /** Timestamp of when ring started to not be sensed. -1 means ring was
  sensed
   * previously. */
  static int startedNotSensingRing = -1;
  intake.m_motors.move(POWER);
  // Wait until no longer sensing ring to increment mogo rings.
  // Might need a filter to ensure rings aren't counted twice.
  bool sensedRing = intake.getSensedRing() == color;
  if (!sensedRing && startedNotSensingRing == -1)
    startedNotSensingRing = pros::millis();
  else if (!sensedRing && pros::millis() - startedNotSensingRing > 50) {
    intake.setState(afterState);
    intake.numOfMogoRings++;
    intake.numOfRings++;
    printf("%ith ring scored on mogo!\n", intake.numOfMogoRings);
  } else if (sensedRing) startedNotSensingRing = -1;

  // intake.m_motors.move(POWER);
  // // Wait until no longer sensing ring to increment mogo rings.
  // // Might need a filter to ensure rings aren't counted twice.
  // if (intake.getSensedRing() != color) {
  //   intake.setState(afterState);
  //   intake.numOfMogoRings++;
  //   intake.numOfRings++;
  // }
}

void Intake::IntakingToMogo::switchDest(Intake& intake, DESTINATION dest) {
  FilteringState::switchDest(intake, dest);
  if (dest == KICK) {
    IntakingToKick newState(afterState, color);
    newState.startTime = this->startTime;
    intake.setState(newState);
  } else if (dest == LB) {
    IntakingToLB newState(afterState, color);
    newState.startTime = this->startTime;
    intake.setState(newState);
  }
}

void Intake::Filtering::update(Intake& intake) {
  intake.m_motors.move(POWER);

  if (pros::millis() % 200 < 10) printFiltering(*this);

  auto maybeColor = intake.getSensedRing();
  if (!maybeColor.has_value()) return;

  COLOR color = maybeColor.value();
  DESTINATION destination;
  switch (color) {
    case COLOR::RED: destination = red; break;
    case COLOR::BLUE: destination = blue; break;
  }

  switch (destination) {
    case MOGO: intake.setState(IntakingToMogo {*this, color}); break;
    case KICK: intake.setState(IntakingToKick {*this, color}); break;
    case LB: intake.setState(IntakingToLB {*this, color}); break;
  }
}

void Intake::Filtering::setDest(COLOR color, DESTINATION dest) {
  switch (color) {
    case COLOR::RED: red = dest; break;
    case COLOR::BLUE: blue = dest; break;
  }
}

void Intake::Filtering::start(Intake& intake) {
  Intake::BaseState::start(intake);
   printf("starting filtering\n");
  //intake.m_sensor.set_led_pwm(100);
}

void Intake::Filtering::end(Intake& intake) {
  Intake::BaseState::end(intake);
   printf("ending filtering state\n");
  //intake.m_sensor.set_led_pwm(0);
}

void Intake::Idling::update(Intake& intake) { intake.m_motors.move(0); }

void Intake::Outtaking::update(Intake& intake) { intake.m_motors.move(-POWER); }

void Intake::update() {
  const State prevState = m_state;

  std::visit(m_visitor, m_state);

  // if state changed types, run update again
  if (prevState.index() != m_state.index()) update();
}

void Intake::setState(State newState) {
  if (newState.index() == m_state.index()) return;
  std::visit([this](BaseState& state) { state.end(*this); }, m_state);
  m_state = newState;
  std::visit([this](BaseState& state) { state.start(*this); }, m_state);
  // update();
}

void Intake::stop() { setState(Idling()); }

void Intake::outtake() { setState(Outtaking()); }

template <class... Ts> struct overloads : Ts... {
    using Ts::operator()...;
};
template <class... Ts> overloads(Ts...) -> overloads<Ts...>;

void Intake::intakeTo(COLOR color, DESTINATION dest) {
  // todo: does this get built for every call to this function? (performance?)
  auto visitor = overloads(
      [this, dest, color](FilteringState& state) {
        if (state.color == color) state.switchDest(*this, dest);
        else state.afterState.setDest(color, dest);
      },
      [this, dest, color](Filtering& state) { state.setDest(color, dest); },
      [this, dest, color](BaseState& state) {
        Filtering newState {};
        newState.setDest(color, dest);
        setState(newState);
      });
  std::visit(visitor, m_state);
}

void Intake::intakeBothTo(DESTINATION dest) {
  intakeTo(COLOR::RED, dest);
  intakeTo(COLOR::BLUE, dest);
}

Intake::Sensor::Sensor(pros::Optical& optical) : m_optical(optical) {
  m_optical.set_led_pwm(100);
}

std::optional<COLOR> Intake::Sensor::getRing() const {
  // if (pros::millis() % 500 < 10)
  //   printf("proximity: %d\n", m_optical.get_proximity());

  if (m_optical.get_proximity() < 200) return std::nullopt;
  const float hueRem = std::remainder(m_optical.get_hue(), 360);
  printf("hueRem: %f\n", hueRem);
  if (std::abs(hueRem) < 60) return COLOR::RED; printf("red!");
  printf("blue!");
  return COLOR::BLUE;
}

std::optional<COLOR> Intake::getSensedRing() { return m_sensor.getRing(); }