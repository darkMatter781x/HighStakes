#include "kalman.h"

KalmanOdom::Measurement::Matrix KalmanOdom::Measurement::matrix() const {
  Matrix m;
  m << leftDriveTravel.internal(), rightDriveTravel.internal(),
      vertTravel.internal(), horiTravel.internal(), thetaAccel.internal();
  return m;
}

KalmanOdom::ControlInput::Matrix KalmanOdom::ControlInput::matrix() const {
  Matrix m;
  m << leftVoltage.internal(), rightVoltage.internal();
  return m;
}

KalmanOdom::State& KalmanOdom::stateTransition(State& x, Time deltaT) {
  return x;
}