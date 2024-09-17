#include "leds/power.h"
#include "subsystems.h"

void LedPowerHandler::update() {
  for (auto i = m_strips.begin(); i != m_strips.end();) {
    // Get the range of the current key
    const auto range = m_strips.equal_range(i->first);

    // The desired current for this expander without any reduction
    float expanderCurrent = 0;
    // std::vector<float> stripCurrents;
    // Sum the current of all strips on this expander
    for (auto j = range.first; j != range.second; ++j) {
      auto strip = j->second;
      const float current = strip->getCurrent();

      // if (pros::millis() % 100 < 10)
        // printf("⇃ %ipx, %fW\n", strip->size(), current);
      // stripCurrents.push_back(current);
      expanderCurrent += current;
    }

    const float reduction = std::min(currentLimit / expanderCurrent, 1.0f);
    // Reduce the brightness of all strips on this expander
    for (auto j = range.first; j != range.second; ++j) {
      auto strip = j->second;
      // If wouldn't over-current, then don't reduce the brightness
      strip->applyReduction(reduction);
    }
    if (pros::millis() % 100 < 10) printf("reduction: %f\n", reduction);
    // Move to the next key
    i = range.second;
  }
}

void LedPowerHandler::registerStrip(std::shared_ptr<LedStrip> strip) {
  m_strips.insert({strip->getADIExpander(), strip});
};

LedPowerHandler* LedPowerHandler::instance = nullptr;

LedPowerHandler& LedPowerHandler::get() {
  if (LedPowerHandler::instance == nullptr)
    LedPowerHandler::instance = new LedPowerHandler();
  return *LedPowerHandler::instance;
}

LedPowerHandler::LedPowerHandler() : Subsystem(), m_strips() {}

const float LedPowerHandler::currentLimit = 2;