#include "subsystems/selector.h"
#include "liblvgl/llemu.hpp"
#include "pros/error.h"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "robot.h"
#include "subsystems.h"
#include "subsystems/intake.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>

void func(ALLIANCE) {}

std::vector<Auton> AutonSelector::autons {
    Auton {.label = (char*)"adfs",
           .run = func,
           .labelForController = std::nullopt},
    Auton {.label = (char*)"adfs2",
           .run = func,
           .labelForController = std::nullopt},
    Auton {.label = (char*)"adfs3",
           .run = func,
           .labelForController = std::nullopt}};

AutonSelector* AutonSelector::instance = nullptr;

bool didConstructorRun = false;

AutonSelector::AutonSelector(pros::adi::Potentiometer& pot,
                             Intake::Sensor& intakeSensor)
  : m_pot(pot), m_intakeSensor(intakeSensor), m_alliance(ALLIANCE::RED),
    m_autons {Auton {.label = (char*)"adfs",
                     .run = func,
                     .labelForController = std::nullopt},
              Auton {.label = (char*)"adfs2",
                     .run = func,
                     .labelForController = std::nullopt},
              Auton {.label = (char*)"adfs3",
                     .run = func,
                     .labelForController = std::nullopt}},
    m_indexOffset(0), m_locked(std::nullopt) {
  pros::lcd::initialize();

  printf("print m_autons\n");
  for (auto a : m_autons) { printf("\t%s,\n", a.label); }
  printf("end m_autons\n");
  printf("print AutonSelector::autons\n");
  for (auto a : AutonSelector::autons) { printf("\t%s,\n", a.label); }
  printf("end AutonSelector::autons\n");

  // Register the brain screen button callbacks.
  pros::lcd::register_btn0_cb([] { AutonSelector::getPtr()->increment(); });
  pros::lcd::register_btn1_cb(
      [] { AutonSelector::getPtr()->toggleAlliance(); });
  pros::lcd::register_btn2_cb([] { AutonSelector::getPtr()->decrement(); });
}

void AutonSelector::increment() {
  int newIndex = getIndex() + 1;
  if (newIndex > m_autons.size()) newIndex = 0;
  setIndex(newIndex);
}

void AutonSelector::decrement() {
  int newIndex = getIndex() - 1;
  if (newIndex < 0) newIndex = m_autons.size() - 1;
  setIndex(newIndex);
}

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
  int val = m_pot.get_value();
  if (val == PROS_ERR) return 0;
  val = std::max(std::min(val, 4096), 0);
  return (val * m_autons.size()) / 4096;
}

const Auton& AutonSelector::getAuton() const { return m_autons.at(getIndex()); }

void AutonSelector::runAuton() { getAuton().run(getAlliance()); }

const ALLIANCE& AutonSelector::getAlliance() const {
  if (m_locked.has_value()) return m_locked->alliance;
  else return m_alliance;
}

void AutonSelector::setIndex(size_t newIndex) {
  m_indexOffset = newIndex - getSensedIndex();
  if (newIndex != getIndex()) {
    printf("AutonSelector::setIndex() failed to set index!\n");
    printf("\tnewIndex: %i\n", newIndex);
    printf("\tsensedIndex(): %i\n", getSensedIndex());
    printf("\tgetIndex(): %i\n", getIndex());
  }
}

void AutonSelector::lock() {
  m_locked = Selection {.index = getIndex(), .alliance = getAlliance()};
}

void AutonSelector::update() {
  // pros::delay(1000);
  // printf("did construct?: %i\n", didConstructorRun);
  // printf("did ignore sensor?: %i\n", m_ignoreSensor);
  // printf("print m_autons\n");
  // for (auto a : m_autons) { printf("\t%s,\n", a.label); }
  // printf("end m_autons\n");
  // printf("print AutonSelector::autons\n");
  // for (auto a : AutonSelector::autons) { printf("\t%s,\n", a.label); }
  // printf("end AutonSelector::autons\n");
  // pros::delay(1000);

  // if (!pros::competition::is_disabled()) {
  //   // SubsystemHandler::get()->removeSubsystem(m_id);
  //   lock();
  // }

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