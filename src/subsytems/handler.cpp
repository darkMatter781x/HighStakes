#include "pros/rtos.hpp"
#include "subsystems.h"
#include <sys/_intsup.h>

SubsystemHandler::SubsystemHandler()
  : m_lastUsedId(0), m_subsystems {}, m_task {[this] {
      unsigned int now = pros::millis();
      while (true) {
        for (auto sub : m_subsystems) sub.second->update();
        pros::c::task_delay_until(&now, 10);
      }
    }} {}

SubsystemHandler* SubsystemHandler::get() { return SubsystemHandler::instance; }

int SubsystemHandler::addSubsystem(Subsystem* subsystem) {
  m_subsystems.emplace(++m_lastUsedId, subsystem);
  return m_lastUsedId;
}

void SubsystemHandler::removeSubsystem(int subsystemId) {
  m_subsystems.erase(subsystemId);
}