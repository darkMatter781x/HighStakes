#include "subsystems/selector.h"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "robot.h"
#include "subsystems.h"
#include "subsystems/intake.h"

AutonSelector::AutonSelector(pros::adi::Potentiometer& pot,
                             Intake::Sensor& intakeSensor)
  : m_pot(pot), m_intakeSensor(intakeSensor), m_alliance(ALLIANCE::RED),
    m_autons(AutonSelector::autons), m_indexOffset(0), m_locked(std::nullopt) {
  pros::lcd::initialize();

  // Register the brain screen button callbacks.
  pros::lcd::register_btn0_cb([] { AutonSelector::getPtr()->increment(); });
  pros::lcd::register_btn1_cb(
      [] { AutonSelector::getPtr()->toggleAlliance(); });
  pros::lcd::register_btn2_cb([] { AutonSelector::getPtr()->decrement(); });
}

void AutonSelector::increment() { setIndex(getIndex() + 1); }

void AutonSelector::decrement() { setIndex(getIndex() - 1); }

void AutonSelector::toggleAlliance(bool manual) {
  switch (m_alliance) {
    case ALLIANCE::RED: setAlliance(ALLIANCE::BLUE, manual); break;
    case ALLIANCE::BLUE: setAlliance(ALLIANCE::RED, manual); break;
  }
}

void AutonSelector::setAlliance(ALLIANCE alliance, bool manual) {
  if (manual) m_ignoreSensor = true;
  m_alliance = alliance;
}

size_t AutonSelector::getIndex() const {
  if (m_locked.has_value()) return m_locked->index;
  else return getSensedIndex() + m_indexOffset;
}

size_t AutonSelector::getSensedIndex() const {
  return m_pot.get_value() / (4096 / m_autons.size());
}

const Auton& AutonSelector::getAuton() const { return m_autons[getIndex()]; }

void AutonSelector::runAuton() { getAuton().run(getAlliance()); }

const ALLIANCE& AutonSelector::getAlliance() const {
  if (m_locked.has_value()) return m_locked->alliance;
  else return m_alliance;
}

void AutonSelector::setIndex(size_t index) {
  m_indexOffset = index - getSensedIndex();
}

void AutonSelector::lock() {
  m_locked = Selection {.index = getIndex(), .alliance = getAlliance()};
}

void AutonSelector::update() {
  if (!pros::competition::is_disabled()) {
    SubsystemHandler::get()->removeSubsystem(m_id);
    lock();
  }

  if (!m_ignoreSensor)
    setAlliance(m_intakeSensor.getRing().value_or(getAlliance()), false);
  display();
}

void AutonSelector::display() const {
  const Auton& auton = getAuton();
  const ALLIANCE& alliance = getAlliance();

  // Display the auton name on the controller.
  bot.gamepad.print(0, 0, "%s", auton.labelForController.value_or(auton.label));

  // Display the alliance and auton name on the lcd.
  pros::lcd::print(0, "%s", auton.label);
  pros::lcd::print(1, "%s", alliance == ALLIANCE::RED ? "RED" : "BLUE");

  // Display the alliance color on the LEDs
  // TODO
}