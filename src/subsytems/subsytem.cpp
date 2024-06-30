#include "subsystems.h"

Subsystem::Subsystem() : m_id(SubsystemHandler::get()->addSubsystem(this)) {}

Subsystem::~Subsystem() { SubsystemHandler::get()->removeSubsystem(m_id); }