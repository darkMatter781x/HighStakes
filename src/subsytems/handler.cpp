#include "pros/rtos.hpp"
#include "subsystems.h"
#include <sys/_intsup.h>

#define ANSI_RED "\x1b[31m"
#define ANSI_RESET "\x1b[0m"

SubsystemHandler::SubsystemHandler()
  : m_lastUsedId(0), m_subsystems {}, m_task {[this] {
      uint32_t now = pros::millis();
      while (true) {
        for (auto sub : m_subsystems) try {
            if (sub.second != nullptr) sub.second->update();
          } catch (std::exception err) {
            printf(ANSI_RED
                   "Removing subsystem due to error (id=%i): %s" ANSI_RESET
                   "\n",
                   sub.first, err.what());
            m_subsystems.erase(sub.first);
          }
        pros::c::task_delay_until(&now, 10);
      }
    }, 1, 0xF000} {}

SubsystemHandler* SubsystemHandler::instance;

SubsystemHandler* SubsystemHandler::get() {
  if (SubsystemHandler::instance == nullptr)
    SubsystemHandler::instance = new SubsystemHandler();
  return SubsystemHandler::instance;
}

int SubsystemHandler::addSubsystem(Subsystem* subsystem) {
  m_subsystems.emplace(++m_lastUsedId, subsystem);
  return m_lastUsedId;
}

void SubsystemHandler::removeSubsystem(int subsystemId) {
  printf("subsystem remove\n");
  m_subsystems.erase(subsystemId);
}