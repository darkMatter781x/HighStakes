#include "subsystems/intake.h"

Intake::Intake(pros::MotorGroup& motors, pros::Optical& optical,
               pros::adi::Pneumatics& kicker, Config conf)
  : m_motors(motors), m_optical(optical), m_kicker(kicker),
    m_visitor([this](BaseState& state) { state.update(*this); }), m_conf(conf) {
  m_optical.set_led_pwm(100);
}

Intake::Config Intake::Config::config = {1000, 1000, 1000, 1000};

const Intake::State& Intake::getState() const { return m_state; }

Intake::FilteringState::FilteringState(Filtering afterState, COLOR color)
  : startTime(pros::millis()), color(color), afterState(afterState) {}

void Intake::FilteringState::switchDest(Intake& intake, DESTINATION dest) {
  afterState.setDest(color, dest);
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
  if (pros::millis() - startTime > intake.m_conf.intakingToKickDuration)
    intake.setState(IdlingToKick {afterState, color});
}

void Intake::IntakingToKick::switchDest(Intake& intake, DESTINATION dest) {
  FilteringState::switchDest(intake, dest);
  if (dest == LIFT) {
    IntakingToLift newState(afterState, color);
    newState.startTime = this->startTime;
    intake.setState(newState);
  } else if (dest == MOGO) {
    intake.setState(Filtering {afterState});
  }
}

void Intake::IdlingToKick::update(Intake& intake) {
  intake.m_motors.move(0);
  intake.m_kicker.set_value(true);
  if (pros::millis() - startTime > intake.m_conf.idlingToKickDuration) {
    intake.m_kicker.set_value(false);
    intake.setState(afterState);
  }
}

void Intake::IntakingToLift::update(Intake& intake) {
  intake.m_motors.move(127);
  if (pros::millis() - startTime > intake.m_conf.intakingToLiftDuration)
    intake.setState(OuttakingToLift {afterState, color});
}

void Intake::OuttakingToLift::update(Intake& intake) {
  intake.m_motors.move(-127);
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

void Intake::Idling::update(Intake& intake) { intake.m_motors.move(0); }

void Intake::Outtaking::update(Intake& intake) { intake.m_motors.move(-127); }

void Intake::update() {
  const State prevState = m_state;

  std::visit(m_visitor, m_state);

  // if state changed types, run update again
  if (prevState.index() != m_state.index()) update();
}

void Intake::setState(State state) {
  m_state = state;
  update();
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
        m_state = newState;
      });
  std::visit(visitor, m_state);
}

void Intake::intakeBothTo(DESTINATION dest) {
  intakeTo(RED, dest);
  intakeTo(BLUE, dest);
}

std::optional<Intake::COLOR> Intake::getSensedRing() {
  if (m_optical.get_proximity() < 128) return std::nullopt;
  // todo: check
  if (m_optical.get_hue() < 100) return RED;
  return BLUE;
}