#pragma once

#include "intake.h"
#include "pros/adi.hpp"
#include "subsystems.h"
#include <functional>
#include <optional>
#include "color.h"
#include "auton.h"

class AutonSelector : Subsystem {
  private:
    struct Selection {
        size_t index;
        ALLIANCE alliance;
    };

    std::vector<auton::Auton> m_autons;

    ALLIANCE m_alliance;
    /** @brief Amount of offset from sensed index. */
    size_t m_indexOffset;

    /**
     * @brief The locked in selection. Once set, the selection cannot be
     * changed.
     */
    std::optional<Selection> m_locked;

    /** @brief Potentiometer for selecting autons. */
    pros::adi::Potentiometer& m_pot;
    /** @brief Intake sensor used to automatically determine alliance color. */
    Intake::Sensor& m_intakeSensor;

    /**
     * @brief Prevents the intake sensor from changing the selected alliance.
     * Set to true if alliance modified with screen.
     */
    bool m_ignoreSensor;

    /** @brief Sets the selected auton to the desired index. */
    void setIndex(size_t index);

    void increment();
    void decrement();

    /**
     * @brief Changes currently selected alliance.
     *
     * @param manual Determines whether m_ignoreSensor is set to true.
     */
    void setAlliance(ALLIANCE alliance, bool manual = true);
    /**
     * @brief Toggles the currently selected alliance.
     *
     * @param manual Determines whether m_ignoreSensor is set to true.
     */
    void toggleAlliance(bool manual = true);

    /** @returns The index sensed by the potentiometer. */
    size_t getSensedIndex() const;

    /** @brief Updates the display. */
    void display() const;

    AutonSelector(pros::adi::Potentiometer& pot, Intake::Sensor& intakeSensor);
    static AutonSelector* instance;
  public:
    /** @returns index of selected auton */
    size_t getIndex() const;
    /** @returns selected auton */
    const auton::Auton& getAuton() const;

    /** Runs selected auton */
    void runAuton();

    /** @returns Selected alliance */
    const ALLIANCE& getAlliance() const;

    /** Locks auton in for duration of program. */
    void lock();
    /** @returns Whether auton has been locked in. */
    bool isLocked() const;

    void update() override;

    inline static AutonSelector& getRef(pros::adi::Potentiometer& pot,
                                        Intake::Sensor& intakeSensor) {
      if (instance == nullptr) instance = new AutonSelector(pot, intakeSensor);
      return *instance;
    }

    /** Gets a potentially null pointer to the selector */
    inline static AutonSelector* getPtr() { return instance; }
};

// if comp plugged in -> e