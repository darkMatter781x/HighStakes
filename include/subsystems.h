#include "pros/rtos.hpp"
#include <map>

#pragma once

/**
 * @brief A Subsystem should provides an abstracted interface for controlling
 * said subsystem.
 * If needed, an update method can be written that will be ran
 * every 10ms by the SubsystemHandler.
 */
class Subsystem {
  protected:
    /** @brief id for this Subsystem, determined by the SubsystemHandler. */
    const int m_id;

    /** @brief Automatically adds this Subsystem to the SubsystemHandle. */
    Subsystem();
  public:
    /** @brief Is run every 10ms by the SubsystemHandler. */
    virtual void update() = 0;

    /** @brief Removes this Subsystem from the SubsystemHandler. As it should
     * never be called, it will create a log message. */
    virtual ~Subsystem();
};

/**
 * @brief Handles Subsystems, calling their update() method every 10ms.
 * Follows the Singleton pattern
 */
class SubsystemHandler {
  public:
    /**
     * @brief Gets the SubsystemHandler instance.
     * If it has not been previously constructed (instance == nullptr), then
     * this method will construct it.
     */
    static SubsystemHandler* get();

    /**
     * @brief Adds subsystem to SubsystemHandler's vector, causing it to be
     * updated every 10ms.
     *
     * @return int Id for the subsystem which can be used to remove it from the
     * SubsystemHandler.
     */
    int addSubsystem(Subsystem* subsystem);

    /**
     * @brief Removes the subsytem from the SubsystemHandler's vector,
     * preventing it from being updated every 10ms.
     *
     * @param subsystemId the id returned from addSubsystem.
     */
    void removeSubsystem(int subsystemId);
  private:
    /** @brief the last id used to add a Subsystem. */
    int m_lastUsedId;
    /** @brief map of id to Subsystem. */
    std::unordered_map<int, Subsystem*> m_subsystems;
    /** @brief the task responsible for updating m_subsystems every 10ms. */
    pros::Task m_task;

    /** @brief updates each of the m_subsystems. */
    void update();

    /** @brief starts the handler task */
    SubsystemHandler();
    /**
     * @brief Should ever be one instance of SubsystemHandler, and that's this
     * one.
     */
    static SubsystemHandler* instance;
};