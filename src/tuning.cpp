#include "tuning.h"
#include "pros/misc.h"
#include "robot.h"
#include <cmath>

std::vector<std::string> split(const std::string& _input,
                               const std::string& delimiter) {
  std::vector<std::string> tokens;
  std::string source = _input;
  size_t pos = 0;
  while ((pos = source.find(delimiter)) != std::string::npos) {
    tokens.push_back(source.substr(0, pos));
    source.erase(0, pos + delimiter.length());
  }
  tokens.push_back(source);
  return tokens;
}

void makeLowerCase(std::string& str) {
  std::transform(str.begin(), str.end(), str.begin(),
                 [](unsigned char c) { return std::tolower(c); });
}

enum TUNING_MODE { LATERAL, ANGULAR };

float getValue(const TUNING_MODE mode) {
  if (mode == LATERAL) return bot.getPose().y;
  return bot.getPose().theta;
}

void print(const TUNING_MODE mode, bool key = true) {
  if (mode == LATERAL) printf("%s%fin\n", key ? "y: " : "", getValue(mode));
  else printf("%s%fdeg\n", key ? "theta: " : "", getValue(mode));
  printf("battery: %i\n", int(pros::battery::get_capacity()));

  // print average drive motor temperature
  auto temps = bot.m_config.motors.left.get_temperature_all();
  auto rightTemps = bot.m_config.motors.right.get_temperature_all();
  // concat right motor temps to temps
  temps.insert(temps.end(), rightTemps.begin(), rightTemps.end());
  // sum temps
  float sum = 0;
  for (const auto& temp : temps) sum += temp;
  float average = sum / temps.size();
  printf("motor temp: %f\n", average);
}

void tuningCLI() {
  static TUNING_MODE mode = ANGULAR;
  lemlib::PID* pid = &(mode == LATERAL ? bot.lateralPID : bot.angularPID);
  lemlib::ControllerSettings* settings =
      &(mode == LATERAL ? bot.lateralSettings : bot.angularSettings);

  const float defaultAngularDist = 90;
  const float defaultLateralDist = 24;
  float angularDist = defaultAngularDist;
  float lateralDist = defaultLateralDist;
  while (1) {
    pros::Controller gamepad(pros::E_CONTROLLER_MASTER);
    try {
      std::cout << "pid tuner> ";
      std::string input;
      getline(std::cin, input);
      makeLowerCase(input);
      auto params = split(input, " ");
      std::string command = params.at(0);

      if (command == "s" || command == "set") {
        if (params.size() < 3) {
          std::cout << "invalid number of arguments" << std::endl;
          continue;
        }
        std::string gainType = params.at(1);
        std::string gainValueStr = params.at(2);
        float gainValue = std::stof(gainValueStr);

        if (gainType.find("p") != std::string::npos) {
          pid->kP = gainValue;
        } else if (gainType.find("d") != std::string::npos) {
          pid->kD = gainValue;
        } else if (gainType.find("i") != std::string::npos) {
          pid->kI = gainValue;
        } else if (gainType.find("s") != std::string::npos) {
          settings->slew = gainValue;
        } else {
          std::cout << "invalid gain type" << std::endl;
        }
      } else if (command == "g" || command == "get") {
        if (params.size() < 2) {
          std::cout << "invalid number of arguments" << std::endl;
          continue;
        }
        std::string gainType = params.at(1);
        if (gainType == "mode") {
          std::cout << "mode: " << (mode == LATERAL ? "lateral" : "angular")
                    << std::endl;
        } else if (gainType == "dist") {
          std::cout << "dist: " << (mode == LATERAL ? lateralDist : angularDist)
                    << std::endl;
        } else if (gainType.find("p") != std::string::npos) {
          std::cout << "kP: " << pid->kP << std::endl;
        } else if (gainType.find("d") != std::string::npos) {
          std::cout << "kD: " << pid->kD << std::endl;
        } else if (gainType.find("i") != std::string::npos) {
          std::cout << "kI: " << pid->kI << std::endl;
        } else if (gainType.find("s") != std::string::npos) {
          std::cout << "slew: " << settings->slew << std::endl;
        } else {
          std::cout << "invalid gain type" << std::endl;
        }
      } else if (command == "run" || command == "x" || command == "rr") {
        bot.cancelMotion();
        bot.setPose(0, 0, 0);
        float timeout = 2000;
        bool wait = true;
        if (params.size() > 1) {
          auto noWaitIt = find(params.begin(), params.end(), "-n");
          if (noWaitIt != params.end()) {
            timeout = 1000000;
            wait = false;
          }

          auto timeoutIt = find(params.begin(), params.end(), "-t");
          if (timeoutIt != params.end() && ++timeoutIt != params.end()) {
            timeout = std::stof(*timeoutIt);
          }
        }
        const bool reversed = command == "rr";
        const int multiplier = reversed ? -1 : 1;
        const int startTime = pros::millis();
        switch (mode) {
          case LATERAL:
            bot.moveToPoint(0, multiplier * lateralDist, timeout,
                            {.forwards = !reversed});
            break;
          case ANGULAR:
            bot.turnToHeading(multiplier * angularDist, timeout);
            break;
        }

        if (wait) {
          float prev = getValue(mode);
          float prevOscillation = 0;
          float prevVel = 0;
          int count = 0;

          printf("oscillations\nnum\ttime\tcurr\taccel\n");
          while (bot.isInMotion() &&
                 !gamepad.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            pros::delay(10);
            const float curr = getValue(mode);
            const float vel = (curr - prev) / 0.01;
            const float smoothVel = vel * 0.75 + prevVel * 0.25;
            prev = curr;

            // if sign of velocity changes, and this oscillation is not small
            if (smoothVel * prevVel < 0 && std::fabs(prevOscillation - curr) >
                                               (mode == LATERAL ? 0.25 : 1)) {
              printf("%i\t%4.2f\t%4.2f\t%4.2f\n", ++count,
                     float(pros::millis() - startTime) / 1000, curr,
                     smoothVel - prevVel);
              prevOscillation = curr;
            }
            prevVel = vel;
          }
          printf("\n");
          print(mode);
        }
      } else if (command == "print" || command == "p") {
        print(mode);
      } else if (command == "stop" || command == "s") {
        bot.cancelMotion();
      } else if (command == "exit") {
        break;

      } else if (command == "mode") {
        if (params.size() < 2) {
          std::cout << "invalid number of arguments" << std::endl;
          continue;
        }
        std::string newMode = params.at(1);

        if (newMode.find("l") != std::string::npos) {
          std::cout << "switching to lateral mode" << std::endl;
          pid = &bot.lateralPID;
          settings = &bot.lateralSettings;
          mode = LATERAL;
        } else if (newMode.find("a") != std::string::npos) {
          std::cout << "switching to angular mode" << std::endl;
          mode = ANGULAR;
          pid = &bot.angularPID;
          settings = &bot.angularSettings;
        } else {
          std::cout << "invalid mode" << std::endl;
        }

      } else if (command == "dist") {
        if (params.size() < 2) {
          std::cout << "invalid number of arguments" << std::endl;
          continue;
        }
        float dist;
        if (std::string(params.at(1)).find("r") != std::string::npos)
          dist = mode == LATERAL ? defaultLateralDist : defaultAngularDist;
        dist = std::stof(params.at(1));

        printf("setting %s dist to: %f\n",
               mode == LATERAL ? "lateral" : "angular", dist);

        if (mode == LATERAL) {
          lateralDist = dist;
        } else {
          angularDist = dist;
        }

      } else {
        std::cout << "invalid command" << std::endl;
      }
    } catch (std::exception e) {
      std::cout << "error: " << e.what() << std::endl;
    }

    pros::delay(10);
  }
}

const bool tuneModeEnabled = false;