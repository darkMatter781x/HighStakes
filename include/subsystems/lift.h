#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/exitcondition.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "subsystems.h"

class Lift : Subsystem {
  public:
    enum State {
      /** low enough to pick up ring from mogo */
      BOTTOM,
      /** height to score in mogo and alliance wall stake*/
      MIDDLE,
      /** height to score on a wall stake */
      TOP,
      /** brakes motor in case of emergency */
      EMERGENCY_STOP,
    };

    struct Config {
        /** target angle of lift for State::BOTTOM. 0 is highest possible angle
         * and down is positive */
        float bottom;
        /** target angle of lift for State::MIDDLE. 0 is highest possible angle
         * and down is positive */
        float middle;
        /** target angle of lift for State::TOP. 0 is highest possible angle and
         * down is positive */
        float top;
        /** gear ratio of lift : rotation sensor */
        float gearRatio;

        /** controller settings for lift */
        lemlib::ControllerSettings controllerSettings;

        /** default config */
        static Config config;
    };
  private:
    State m_state;
    Config& m_config;
    pros::MotorGroup& m_motors;
    pros::Rotation& m_rotation;

    lemlib::PID m_pid;
    /** small exit condition */
    lemlib::ExitCondition m_exitCondition;

    /**
     * @returns The target lift angle in degrees based on m_state and m_config.
     * @returns NaN if current state is EMERGENCY_STOP.
     */
    float getTargetAngle() const;
    /** @returns Current angle of the lift in degrees. */
    float calcLiftAngle() const;
    /** @returns Error between current lift angle and target lift angle.  */
    float calcError() const;
  public:
    Lift(pros::MotorGroup& motors, pros::Rotation& rotation, Config& config);

    void update() override;

    const State& getState();
    void setState(State state);

    void emergencyStop();
    void goToBottom();
    void goToMiddle();
    void goToTop();

    /** Moves lift to the next highest state. If m_state is EMERGENCY_STOPPED,
     * then don't change m_state. */
    void goUp();
    /** Moves lift to the next lowest state. If m_state is EMERGENCY_STOPPED,
     * then don't change m_state. */
    void goDown();
};