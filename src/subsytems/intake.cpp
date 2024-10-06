#include "subsystems/intake.h"
#include "pros/motors.h"
#include <cmath>
#include <cstdio>

// TODO: use mutex for state changes

std::string destToStr(Intake::DESTINATION dest) {
  switch (dest) {
    case Intake::MOGO: return "MOGO";
    case Intake::KICK: return "KICK";
    case Intake::LIFT: return "LIFT";
  }
}

void printFiltering(Intake::Filtering& filter) {
  printf("filtering:\tr:\t%s\tb:\t%s\n", destToStr(filter.red).c_str(),
         destToStr(filter.blue).c_str());
}

Intake::Intake(pros::MotorGroup& motors, pros::Optical& optical,
               pros::adi::Pneumatics& kicker, Config conf)
  : m_motors(motors), m_optical(optical), m_kicker(kicker),
    m_visitor([this](BaseState& state) { state.update(*this); }), m_conf(conf) {
  m_motors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
}

const Intake::State& Intake::getState() const { return m_state; }

void Intake::BaseState::start(Intake& intake) { printf("starting state\n"); }

void Intake::BaseState::end(Intake& intake) { printf("ending state\n"); }

Intake::FilteringState::FilteringState(Filtering afterState, COLOR color)
  : color(color), afterState(afterState) {}

void Intake::FilteringState::switchDest(Intake& intake, DESTINATION dest) {
  afterState.setDest(color, dest);
}

float Intake::FilteringState::getInchesTraveled(const Intake& intake) const {
  printf("getting inches traveled, pos: %4.2f\n", intake.m_motors.get_position() - startPosition);
  return (intake.m_motors.get_position() - startPosition) /* rots */ /
         (intake.m_conf.chainRatio /* rots per inch */);
}

void Intake::FilteringState::start(Intake& intake) {
  Intake::BaseState::start(intake);
  printf("starting filtering state\n");
  startTime = pros::millis();
  startPosition = intake.m_motors.get_position();
  intake.m_optical.set_led_pwm(100);
}

void Intake::FilteringState::end(Intake& intake) {
  Intake::BaseState::end(intake);
  printf("ending filtering state\n");
  intake.m_optical.set_led_pwm(0);
}

Intake::IntakingToKick::IntakingToKick(Filtering afterState, COLOR color)
  : FilteringState(afterState, color) {}

Intake::IdlingToKick::IdlingToKick(Filtering afterState, COLOR color)
  : FilteringState(afterState, color) {}

Intake::IntakingToLift::IntakingToLift(Filtering afterState, COLOR color)
  : FilteringState(afterState, color) {}

Intake::OuttakingToLift::OuttakingToLift(Filtering afterState, COLOR color)
  : FilteringState(afterState, color) {}

void Intake::IntakingToKick::update(Intake& intake) {
  intake.m_motors.move(127);
  if (pros::millis() % 200 < 10) printf("intaking to kick\n");
  printFiltering(afterState);
  if (pros::millis() - startTime > intake.m_conf.intakingToKickDuration) {
    printf("naturally switching to idle\n");
    intake.setState(IdlingToKick {afterState, color});
  }
}

void Intake::IntakingToKick::switchDest(Intake& intake, DESTINATION dest) {
  FilteringState::switchDest(intake, dest);
  printf("switch from intaking to kick to %s\n", destToStr(dest).c_str());
  printFiltering(afterState);
  if (dest == LIFT) {
    IntakingToLift newState(afterState, color);
    newState.startTime = this->startTime;
    intake.setState(newState);
  } else if (dest == MOGO) {
    intake.setState(Filtering {afterState});
  }
}

void Intake::IdlingToKick::update(Intake& intake) {
  intake.m_motors.move(127);
  printf("idling to kick\n");
  printFiltering(afterState);
  if (pros::millis() % 200 < 10) printf("idling to kick\n");
  if (pros::millis() - startTime > intake.m_conf.idlingToKickDuration) {
    printf("naturally switching to filtering\n");
    intake.setState(afterState);
  }
}

void Intake::IdlingToKick::start(Intake& intake) {
  Intake::FilteringState::start(intake);
  intake.m_kicker.set_value(true);
}

void Intake::IdlingToKick::end(Intake& intake) {
  Intake::FilteringState::start(intake);
  intake.m_kicker.set_value(false);
}

void Intake::IntakingToLift::update(Intake& intake) {
  intake.m_motors.move(127);
  if (pros::millis() % 200 < 10) printf("intaking to lift\n");
  printf("intaking to lift, %4.2f\n", getInchesTraveled(intake));
  if (getInchesTraveled(intake) > intake.m_conf.intakingToLiftInches)
    intake.setState(OuttakingToLift {afterState, color});
}

void Intake::OuttakingToLift::update(Intake& intake) {
  intake.m_motors.move(-127);
  if (pros::millis() % 200 < 10) printf("outtaking to lift\n");
  if (pros::millis() - startTime > intake.m_conf.outtakingToLiftDuration)
    intake.setState(afterState);
}

void Intake::IntakingToLift::switchDest(Intake& intake, DESTINATION dest) {
  FilteringState::switchDest(intake, dest);
  if (dest == KICK) {
    IntakingToKick newState(afterState, color);
    newState.startTime = this->startTime;
    intake.setState(newState);
  } else if (dest == MOGO) {
    intake.setState(Filtering {afterState});
  }
}

void Intake::Filtering::update(Intake& intake) {
  intake.m_motors.move(127);

  if (pros::millis() % 200 < 10) printFiltering(*this);

  auto maybeColor = intake.getSensedRing();
  if (!maybeColor.has_value()) return;

  COLOR color = maybeColor.value();
  DESTINATION destination;
  switch (color) {
    case RED: destination = red; break;
    case BLUE: destination = blue; break;
  }

  switch (destination) {
    case MOGO: break;
    case KICK: intake.setState(IntakingToKick {*this, color}); break;
    case LIFT: intake.setState(IntakingToLift {*this, color}); break;
  }
}

void Intake::Filtering::setDest(COLOR color, DESTINATION dest) {
  switch (color) {
    case RED: red = dest; break;
    case BLUE: blue = dest; break;
  }
}

void Intake::Filtering::start(Intake& intake) {
  Intake::BaseState::start(intake);
  printf("starting filtering\n");
  intake.m_optical.set_led_pwm(100);
} 

void Intake::Filtering::end(Intake& intake) {
  Intake::BaseState::end(intake);
  printf("ending filtering state\n");
  intake.m_optical.set_led_pwm(0);
}

void Intake::Idling::update(Intake& intake) { intake.m_motors.move(0); }

void Intake::Outtaking::update(Intake& intake) { intake.m_motors.move(-127); }

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
  intakeTo(RED, dest);
  intakeTo(BLUE, dest);
}

std::optional<Intake::COLOR> Intake::getSensedRing() {
  if (pros::millis() % 200 < 10)
    printf("proximity:\t%d\n", m_optical.get_proximity());
  if (m_optical.get_proximity() < 200) return std::nullopt;
  printf("proximity:\t%d\n", m_optical.get_proximity());
  const float hueRem = std::remainder(m_optical.get_hue(), 360);
  printf("hue:\t\t%f\n", hueRem);
  // todo: check
  if (std::abs(hueRem) < 60) return RED;
  return BLUE;
}