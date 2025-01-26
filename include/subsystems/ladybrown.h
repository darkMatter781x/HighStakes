#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/exitcondition.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "subsystems.h"

class LadyBrown : Subsystem {
  public:
    enum State {
      /* Lowest state where the lady brown does not effect the hooks */
      IDLE,
      /* Height where the lady brown picks up the ring from the hooks */
      COLLECT,
      /* Height where the robot is able to intake rings with a ring in the lady brown*/
      IDLE_SCORE,
      /* Height where the you are able to score onto the wall stake by driving
         onto the wall stake */
      SCORE,
      /* Height where the lady brown scores without extra force from the
         drivetrain brakes motor in case of emergency */
      EMERGENCY_STOP,
      /** Coast to touch ladder (for auto) */
      COAST
    };

    struct Config {
        /* target angle of lady brown for State::IDLE */
        float idle;
        /* target angle of lady brown for State::COLLECT */
        float collect;
        /*target angle of lady brown for State::IDLE_COLLECT*/
        float idle_collect;
        /* target angle of lady brown for State::MOVE_SCORE */
        float score;

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
    void goToIDLE();
    void goToCOLLECT();
    void goToSCORE();
    void goToIDLE_SCORE();

    /* cycles the lady brown to the next state */
    void nextState();
};