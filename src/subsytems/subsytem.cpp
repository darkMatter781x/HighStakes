#include "subsystems.h"

Subsystem::Subsystem() : m_id(SubsystemHandler::get()->addSubsystem(this)) {}

Subsystem::~Subsystem() {
  // this should never be run
  printf("destroying subsystem with id of '%i'!\n", m_id);
  SubsystemHandler::get()->removeSubsystem(m_id);
}