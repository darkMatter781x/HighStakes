#include "subsystems/selector.h"
#include "pros/llemu.hpp"
#include "robot.h"

void auton1(ALLIANCE) {
  printf("auton1\n");
  pros::delay(1000);
}
void auton2(ALLIANCE) {
  printf("auton2\n");
  pros::delay(1000);
}
void auton3(ALLIANCE) {
  printf("auton3\n");
  pros::delay(1000);
}


std::vector<Auton> getAutons() {
  return {Auton {.label = (char*)"adfs",
                 .run = auton1,
                 .labelForController = std::nullopt,
                 .description = (char*)"do one thing"},
          Auton {.label = (char*)"adfs2",
                 .run = auton2,
                 .labelForController = std::nullopt,
                 .description = (char*)"do two things"},
          Auton {.label = (char*)"adfs3",
                 .run = auton3,
                 .labelForController = std::nullopt,
                 .description = (char*)"DO THREE THINGS!"}};
}

AutonSelector* AutonSelector::instance = nullptr;

bool didConstructorRun = false;

AutonSelector::AutonSelector(pros::adi::Potentiometer& pot,
                             Intake::Sensor& intakeSensor)
  : m_pot(pot), m_intakeSensor(intakeSensor), m_alliance(ALLIANCE::RED),
    m_autons(getAutons()), m_indexOffset(0), m_locked(std::nullopt),
    m_ignoreSensor(false) {
  pros::lcd::initialize();

  printf("print m_autons\n");
  for (auto a : m_autons) { printf("\t%s,\n", a.label); }
  printf("end m_autons\n");

  // Register the brain screen button callbacks.
  pros::lcd::register_btn0_cb([] { AutonSelector::getPtr()->decrement(); });
  pros::lcd::register_btn1_cb(
      [] { AutonSelector::getPtr()->toggleAlliance(true); });
  pros::lcd::register_btn2_cb([] { AutonSelector::getPtr()->increment(); });
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
  if (manual) {
    printf("setAlliance() called with manual = true -> ignoring sensor now\n");
    m_ignoreSensor = true;
  }
  m_alliance = alliance;
}

size_t AutonSelector::getIndex() const {
  if (m_locked.has_value()) return m_locked->index;
  else return (getSensedIndex() + m_indexOffset) % m_autons.size();
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
  newIndex = newIndex % m_autons.size();

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
  // Prevent changing auton if enabled (match has started).
  if (!pros::competition::is_disabled()) { lock(); }
  // Change alliance based on sensor if it hasn't been overriden by brain
  // screen.
  if (!m_ignoreSensor)
    setAlliance(m_intakeSensor.getRing().value_or(getAlliance()), false);
  // Display changes due to sensor or brain screen.
  display();
}

void AutonSelector::display() const {
  // If the robot is enabled, don't display anything.
  if (!pros::competition::is_disabled()) return;

  const Auton& auton = getAuton();
  const ALLIANCE& alliance = getAlliance();

  const char* allianceStr = alliance == ALLIANCE::RED ? "RED" : "BLUE";

  // Display the auton name on the controller. Do this every 100ms.
  if (pros::millis() % 100 < 10)
    bot.gamepad.print(0, 0, "%s",
                      auton.labelForController.value_or(auton.label));

  // Display the alliance on the controller. Do this every 100ms, offset 50ms
  // from the auton name.
  if ((pros::millis() + 50) % 100 < 10)
    bot.gamepad.print(1, 0, "%s", allianceStr);

  // Display the alliance and auton name on the lcd.
  pros::lcd::print(0, "%s", auton.label);
  pros::lcd::print(1, "%s", allianceStr);
  pros::lcd::print(2, "%s", auton.description);

  // Display the alliance color on the LEDs
  // TODO
}