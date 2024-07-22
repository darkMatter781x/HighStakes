#include "pros/adi.hpp"
#include "subsystems.h"

class MogoClamp : public Subsystem {
  public:
    enum class State { OPEN, CLOSE };
  private:
    State m_state;
    pros::adi::Pneumatics m_piston;
  public:
    MogoClamp(pros::adi::Pneumatics& pistons);

    void close();
    void open();
    void toggle();

    void update() override;

    const State& getState() const;
};