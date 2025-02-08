#include "robot.h"
#include "pf/obstacles.h"
#include "pf/odom/lemlib.h"
#include "pf/odom/sim.h"
#include "pf/sensors/simLaser.h"
#include "pf/sensors/v5laser.h"

lemlib::Drivetrain RobotConfig::makeDrivetrain() const {
  return {&this->motors.left,
          &this->motors.right,
          this->dimensions.trackWidth,
          this->dimensions.driveWheelDiameter,
          this->dimensions.driveWheelRpm,
          this->tunables.horizontalDrift};
}

static lemlib::TrackingWheel* makeTrackingWheel(pros::Rotation* encoder,
                                                float wheelDiameter,
                                                float distance,
                                                float gearRatio = 1) {
  if (encoder->is_installed())
    return new lemlib::TrackingWheel(encoder, wheelDiameter, distance,
                                     gearRatio);
  else return nullptr;
}

lemlib::OdomSensors RobotConfig::makeSensors() const {
  return {
      makeTrackingWheel(&this->sensors.vert, this->dimensions.vertEncDiameter,
                        this->dimensions.vertEncDistance,
                        this->dimensions.vertEncGearRatio),
      nullptr,
      makeTrackingWheel(&this->sensors.hori, this->dimensions.horiEncDiameter,
                        this->dimensions.horiEncDistance,
                        this->dimensions.horiEncGearRatio),
      nullptr, &this->sensors.imu};
}

static std::shared_ptr<Odom> makeRealOdom(const RobotConfig& config) {
  return std::make_shared<LemLibOdom>(config.makeSensors(),
                                      config.makeDrivetrain());
}

static std::shared_ptr<SimOdom> makeSimOdom(const RobotConfig& config) {
  return std::make_shared<SimOdom>();
}

static Localization makeRealLocalization(const RobotConfig& config) {
  auto factory =
      [&bot_conf = config](const Localization::SensorFactoryConfig& sensor_conf)
      -> Localization::SensorVector {
    Localization::SensorVector sensors {};
    for (const auto& laser_conf : bot_conf.sensors.lasers) {
      sensors.push_back(
          std::make_unique<sensors::V5LaserSensor<Localization::N>>(
              sensors::Laser::Config {
                  .pose = laser_conf.pose,
                  .botTheta = sensor_conf.botPose.theta,
                  .obstacles = sensor_conf.obstacles,
              },
              laser_conf.sensor));
    }
    return sensors;
  };
  printf("Make real localization\n");
  return {Localization::Config {
      .odom = makeRealOdom(config),
      .obstacles = getObstacles(),
      .sensorFactory = factory,
      .covar = Eigen::Matrix2f::Identity() * .025,
  }};
}

// TODO: Deduplicate this with makeRealLocalization
static Localization makeSimLocalization(const RobotConfig& config) {
  auto odom = makeSimOdom(config);
  auto factory = [&bot_conf = config,
                  &odom](const Localization::SensorFactoryConfig& sensor_conf)
      -> Localization::SensorVector {
    Localization::SensorVector sensors {};
    for (const auto& laser_conf : bot_conf.sensors.lasers) {
      sensors.push_back(
          std::make_unique<sensors::SimLaserSensor<Localization::N>>(
              sensors::Laser::Config {
                  .pose = laser_conf.pose,
                  .botTheta = sensor_conf.botPose.theta,
                  .obstacles = sensor_conf.obstacles,
              },
              odom->getRealPose()));
    }
    return sensors;
  };
  return {Localization::Config {
      .odom = odom,
      .obstacles = getObstacles(),
      .sensorFactory = factory,
      .covar = Eigen::Matrix2f::Identity() * .05,
  }};
}

static Localization makeLocalization(const RobotConfig& config) {
#ifdef VEXIDE_SIMULATION
  return makeSimLocalization(config);
#else
  return makeRealLocalization(config);
#endif
}

Robot::Robot(const RobotConfig& config)
  : lemlib::Chassis(config.makeDrivetrain(), config.tunables.lateralController,
                    config.tunables.angularController, config.makeSensors(),
                    &config.tunables.driveCurve),
    m_intakeSensor(config.sensors.intake), m_intake {config.motors.intake,
                                                     m_intakeSensor,
                                                     config.pneumatics.kicker,
                                                     Intake::Config::config},
    intake(m_intake), m_lady {config.motors.LadyBrown, config.sensors.lb,
                              LadyBrown::Config::config},
    m_led{config.leds.leftLift, config.leds.rightLift,
          config.leds.leftUnderGlow, config.leds.rightUnderGlow},
    lady(m_lady), m_config(config), led(m_led),
    m_gamepad(pros::E_CONTROLLER_MASTER), gamepad(m_gamepad),
    selector(
        AutonSelector::getRef(config.sensors.autonSelector, m_intakeSensor)),
    m_localization(makeLocalization(config)), localization(m_localization),
    intakeLift(config.pneumatics.intakeLift),
    mogoClamp(config.pneumatics.mogoClamp), doinker(config.pneumatics.doinker),
    m_transform(std::make_shared<auton::AllianceTransform>(COLOR::RED)) {}

Robot Robot::instance {RobotConfig::config};
