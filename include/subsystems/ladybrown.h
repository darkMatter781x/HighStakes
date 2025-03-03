#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/exitcondition.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "subsystems.h"

class LadyBrown : Subsystem {
  public:
    enum State {
      /** Lowest state where the lady brown does not affect the hooks. */
      IDLE,
      /** Angle for picking up rings from the hooks. */
      COLLECT,
      /** Angle where robot can intake with a ring in the lady brown. */
      IDLE_SCORE,
      /** Angle for scoring on a stake. */
      SCORE,
      /** Angle for flipping/tipping a mogo. Goes all the way down. */
      FLIP,
      /** Brakes motor in case of emergency. Only way to exit is with */
      EMERGENCY_STOP,
      /** Coast to touch ladder (for auto) */
      COAST
    };

    struct Config {
        /** target angle of lady brown for State::IDLE */
        float idle;
        /** target angle of lady brown for State::COLLECT */
        float collect;
        /** target angle of lady brown for State::IDLE_SCORE */
        float idle_score;
        /** target angle of lady brown for State::SCORE */
        float score;
        /** target angle of lady brown for State::FLIP */
        float flip;

        /* Controller Settings for lift */
        lemlib::ControllerSettings controllerSettings;

        /* default config */
        static Config config;
    };

    // private:
    State m_state;
    Config& m_config;
    pros::MotorGroup& m_motors;
    pros::Rotation& m_rotation;

    lemlib::PID m_pid;

    lemlib::ExitCondition m_exitCondition;
  public:
    using AngDir = lemlib::AngularDirection;
    AngDir dir = AngDir::AUTO;
    /* returns the target lift angle in degrees based on m_state and m_config */
    float getTargetAngle() const;
    /* returns the current angle of the lady brown in decrees */
    float calcLBAngle() const;
    /** @returns Error between current lady brown angle and target lady brown
     * angle.  */
    float calcError() const;
    LadyBrown(pros::MotorGroup& motors, pros::Rotation& rotation,
              Config& config);

    void update() override;

    const State& getState();
    void setState(State state);

    void emergencyStop();
    void goToIdle();
    void collect();
    void goToIdleScore();
    void score();
    void flip();
};