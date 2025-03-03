/**
 * @brief Helpers for serializing data to an mcap file
 */
#pragma once

#include "foxglove/Log_generated.h"
#include "pf/localization.h"
#include "mcap/writer.hpp"
#include <source_location>
#include <string_view>

namespace logger {

namespace frame {
constexpr std::string FIELD = "field";
}

/** @warning Assumes pose's theta is measured in radians and 0rad is the
 * positive x axis. */
void recordPose(lemlib::Pose pose, const std::string_view channelTopic,
                const std::string_view frameId = frame::FIELD);
void recordParticles(Localization::PF::Data& data,
                     const std::string_view channelTopic,
                     const std::string_view frameId = frame::FIELD);
void log(const std::string_view message, const std::string_view channelTopic,
         foxglove::LogLevel logLevel,
         std::source_location sourceLoc = std::source_location::current());
}; // namespace logger