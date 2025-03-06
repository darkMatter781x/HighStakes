#include "subsystems/selector.h"
#include "auton/autons.h"
#include "pros/llemu.hpp"
#include "robot.h"
#include <algorithm>

using auton::Auton;

std::vector<Auton> getAutons() {
  return {
      auton::autons::negativeElims, auton::autons::negativeRush,
      auton::autons::safePositive,  auton::autons::skills,
      auton::autons::safeAWP,       auton::autons::doNothing,
  };
}

AutonSelector* AutonSelector::instance = nullptr;

bool didConstructorRun = false;

AutonSelector::AutonSelector(pros::adi::Potentiometer& pot,
                             Intake::Sensor& intakeSensor)
  : m_pot(pot), m_intakeSensor(intakeSensor), m_alliance(ALLIANCE::RED),
    m_autons(getAutons()), m_indexOffset(0), m_locked(std::nullopt),
    m_ignoreSensor(false) {
  printf("print m_autons\n");
  for (auto a : m_autons) { printf("\t%s,\n", a.label); }
  printf("end m_autons\n");

#ifndef VEXIDE_SIMULATION
  pros::lcd::initialize();
#endif
  // Register the brain screen button callbacks.
  pros::lcd::register_btn0_cb([] {
    printf("left pressed\n");
    AutonSelector::getPtr()->decrement();
  });
  pros::lcd::register_btn1_cb([] {
    printf("center pressed\n");
    AutonSelector::getPtr()->toggleAlliance(true);
  });
  pros::lcd::register_btn2_cb([] {
    printf("right pressed\n");
    AutonSelector::getPtr()->increment();
  });
  printf("added button callbacks!\n");
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
  printf("AutonSelector::setIndex()\n");
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

/** Wraps long text into multiple strings. */
std::vector<std::string> wrapText(const std::string& text, int lineWidth) {
  std::vector<std::string> lines;
  std::string currentLine;

  size_t i = 0;
  while (i < text.length()) {
    // Find the end of the next word
    size_t wordEnd = text.find(' ', i);
    if (wordEnd == std::string::npos) { wordEnd = text.length(); }

    std::string word = text.substr(i, wordEnd - i);

    // Check if adding the word would exceed the line width
    if (currentLine.length() + word.length() > lineWidth) {
      // Start a new line
      lines.push_back(currentLine);
      currentLine.clear();
    }
    // Add the word to the current line
    currentLine += word + ' ';
    // Advance to the next word
    i = wordEnd + 1;
  }

  // Add the last line, if it isn't empty
  if (!currentLine.empty()) { lines.push_back(currentLine); }

  return lines;
}

void AutonSelector::display() const {
  // If the selector phase has ended (competition was enabled), dont display
  // anything.
  if (isLocked()) return;

  const Auton& auton = getAuton();
  const ALLIANCE& alliance = getAlliance();

  const char* allianceStr = alliance == ALLIANCE::RED ? "RED" : "BLUE";

  std::string firstLine {allianceStr[0]};
  firstLine += " ";
  firstLine += auton.labelForController.value_or(auton.label);

  // Display the auton name on the controller. Do this every 100ms.
  if (pros::millis() % 100 < 10) bot.gamepad.print(0, 0, "%s", firstLine);

  // Display the alliance on the controller. Do this every 100ms, offset 50ms
  // from the auton name.
  if ((pros::millis() + 50) % 100 < 10)
    bot.gamepad.print(1, 0, "%s", allianceStr);

  // Display the alliance and auton name on the lcd.
  pros::lcd::print(0, "%s", auton.label);
  pros::lcd::print(1, "%s", allianceStr);

  auto descriptionVec = wrapText(auton.description, 32);
  static const size_t startDescLine = 3;
  for (size_t screenLine = startDescLine, descLine = 0; screenLine <= 7;
       screenLine++, descLine++) {
    if (descLine >= descriptionVec.size()) pros::lcd::print(screenLine, "");
    else
      pros::lcd::print(screenLine, "%s", descriptionVec.at(descLine).c_str());
  }

  // Display the alliance color on the LEDs
  // TODO
}

bool AutonSelector::isLocked() const { return m_locked.has_value(); }