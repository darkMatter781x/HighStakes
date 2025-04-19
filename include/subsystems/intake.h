#pragma once
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "subsystems.h"
#include <variant>
#include <optional>
#include "color.h"

class Intake : public Subsystem {
  private:
    /** Actually controls motors and implements auto dejam functions */
    class AutoDejamIntake {
      private:
        pros::MotorGroup& m_motors;

        enum STATE {
          /** Do not dejam */
          JUST_INTAKE,
          /** Intake, but attempt to dejam if needed */
          INTAKE_DEJAM,
          /** Outtaking to dejam intake */
          INTAKE_DEJAM_OUT,
          OUTTAKE,
          STOPPED
        } m_state;
      public:
        /** Intake without dejamming */
        void justIntake();
        /** Intake with dejamming */
        void intakeDejam();
        void outtake();
        void stop();

        void update();

        AutoDejamIntake(pros::MotorGroup& motors);
    } intake;
  public:
    /** @brief Wrapper for the intake's optical sensor. */
    class Sensor {
      private:
        pros::Optical& m_optical;
      public:
        Sensor(pros::Optical& optical);
        /** @brief Returns the color of the ring if it is present. */
        std::optional<COLOR> getRing() const;
    };

    /** @brief The destination of the ring. */
    enum DESTINATION { MOGO, KICK, LB };

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

    struct IntakingToLB : public FilteringState {
        DESTINATION dest = LB;

        IntakingToLB(Filtering afterState, COLOR color);
        void update(Intake& intake) override;
        void switchDest(Intake& intake, DESTINATION dest) override;
    };

    struct IntakingToKick : public FilteringState {
        DESTINATION dest = KICK;

        IntakingToKick(Filtering afterState, COLOR color);
        void update(Intake& intake) override;
        void switchDest(Intake& intake, DESTINATION dest) override;
    };

    struct KickingRing : public FilteringState {
        DESTINATION dest = KICK;

        virtual void start(Intake& intake) override;
        virtual void end(Intake& intake) override;
        KickingRing(Filtering afterState, COLOR color);
        void update(Intake& intake) override;
    };

    struct IntakingToMogo : public FilteringState {
        DESTINATION dest = MOGO;

        IntakingToMogo(Filtering afterState, COLOR color);
        void update(Intake& intake) override;
        void switchDest(Intake& intake, DESTINATION dest) override;
    };

    struct Outtaking : public BaseState {
        void update(Intake& intake) override;
    };

    struct Idling : public BaseState {
        void update(Intake& intake) override;
    };

    using State = std::variant<Filtering, IntakingToKick, KickingRing,
                               IntakingToLB, Outtaking, Idling, IntakingToMogo>;

    struct Config {
        float intakingToLiftInches;
        size_t outtakingToLiftDuration;
        float intakingToKickDuration;
        size_t KickingRing;

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
    Sensor& m_sensor;
    pros::adi::Pneumatics& m_kicker;
  public:
    void setState(State state);
    void stop();
    void outtake();

    /** @brief Intake both of the ring colors to dest. */
    void intakeBothTo(DESTINATION dest);
    /** @brief Intake specified color of rings to dest. */
    void intakeTo(COLOR color, DESTINATION dest);

    // std::optional<COLOR> getFilterColor();

    std::optional<COLOR> getSensedRing();

    /* gets the destination of the ring color */
    std::optional<Intake::DESTINATION> getDestination(COLOR color);

    void update() override;

    const State& getState() const;
    // const DESTINATION& getDestination() const;

    /** @brief Number of rings intaked. */
    size_t numOfRings = 0;
    /** @brief Number of rings intaked to lift. */
    size_t numOfLiftRings = 0;
    /** @brief Number of rings intaked to kicker. */
    size_t numOfKickRings = 0;
    /** @brief Number of rings intaked to mogo. */
    size_t numOfMogoRings = 0;

    Intake(pros::MotorGroup& motors, Sensor& sensor,
           pros::adi::Pneumatics& kicker, Config conf);
};
