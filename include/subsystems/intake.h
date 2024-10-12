#pragma once
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "subsystems.h"
#include <variant>

class Intake : public Subsystem {
  public:
    /** @brief Ring color. */
    enum COLOR { RED, BLUE };

    /** @brief The destination of the ring. */
    enum DESTINATION { MOGO, KICK, LIFT };

    struct BaseState {
        virtual void update(Intake& intake) = 0;
        /**
         * @brief Run when intake state is switched to this.
         * Do not run Intake::setState() here, as it may lead to infinite
         * recursion.
         */
        virtual void start(Intake& intake);
        /**
         * @brief Run when intake state is switched from this.
         * Do not run Intake::setState() here, as it may lead to infinite
         * recursion.
         */
        virtual void end(Intake& intake);
    };

    /** @brief Waits until ring is sensed. */
    struct Filtering : public BaseState {
        DESTINATION red = MOGO;
        DESTINATION blue = MOGO;

        void setDest(COLOR color, DESTINATION dest);
        void update(Intake& intake) override;

        virtual void start(Intake& intake) override;
        virtual void end(Intake& intake) override;
    };

    /** @brief State where a ring is being moved based on filter. */
    struct FilteringState : public BaseState {
        size_t startTime;
        float startPosition;
        Filtering afterState;

        /** @brief currently handled ring color */
        COLOR color;

        /** @brief destination of the ring */
        DESTINATION dest;

        /** @return number of inches traveled by instascore chain */
        float getInchesTraveled(const Intake& intake) const;
        /**
         * @brief Attempts to switch destination. May fail if too far along.
         * Updates afterState. */
        virtual void switchDest(Intake& intake, DESTINATION dest);

        virtual void start(Intake& intake) override;
        virtual void end(Intake& intake) override;

        FilteringState(Filtering afterState, COLOR color);
    };

    struct IntakingToLift : public FilteringState {
        DESTINATION dest = LIFT;

        IntakingToLift(Filtering afterState, COLOR color);
        void update(Intake& intake) override;
        void switchDest(Intake& intake, DESTINATION dest) override;
    };

    struct OuttakingToLift : public FilteringState {
        DESTINATION dest = LIFT;

        OuttakingToLift(Filtering afterState, COLOR color);
        void update(Intake& intake) override;
    };

    struct IntakingToKick : public FilteringState {
        DESTINATION dest = KICK;

        IntakingToKick(Filtering afterState, COLOR color);
        void update(Intake& intake) override;
        void switchDest(Intake& intake, DESTINATION dest) override;
    };

    struct IdlingToKick : public FilteringState {
        DESTINATION dest = KICK;

        virtual void start(Intake& intake) override;
        virtual void end(Intake& intake) override;
        IdlingToKick(Filtering afterState, COLOR color);
        void update(Intake& intake) override;
    };

    struct Outtaking : public BaseState {
        void update(Intake& intake) override;
    };

    struct Idling : public BaseState {
        void update(Intake& intake) override;
    };

    using State =
        std::variant<Filtering, IntakingToKick, IdlingToKick, IntakingToLift,
                     OuttakingToLift, Outtaking, Idling>;

    struct Config {
        size_t intakingToLiftInches;
        size_t outtakingToLiftDuration;
        size_t intakingToKickDuration;
        size_t idlingToKickDuration;

        /**
         * @brief ratio of motor rotations to inches traveled by instascore
         * chain
         */
        float chainRatio;

        static Config config;
    };
  private:
    State m_state = Idling();
    const std::function<void(BaseState&)> m_visitor;
    const Config m_conf;

    pros::MotorGroup& m_motors;
    pros::Optical& m_optical;
    pros::adi::Pneumatics& m_kicker;
  public:
    void setState(State state);
    void stop();
    void outtake();

    /** @brief Intake both of the ring colors to dest. */
    void intakeBothTo(DESTINATION dest);
    /** @brief Intake specified color of rings to dest. */
    void intakeTo(COLOR color, DESTINATION dest);

    std::optional<COLOR> getSensedRing();

    void update() override;

    const State& getState() const;

    Intake(pros::MotorGroup& motors, pros::Optical& optical,
           pros::adi::Pneumatics& kicker, Config conf);
};
