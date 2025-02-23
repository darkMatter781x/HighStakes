#include "pf/util/logger.h"

#include "foxglove/Log_generated.h"
#include "foxglove/PointCloud_generated.h"
#include "foxglove/Pose_generated.h"
#include "foxglove/schema.h"

#include "mcap/types.hpp"
#include "mcap/writer.hpp"
#include "pf/localization.h"
#include "profiler/profiler.h"

#include <cstddef>
#include <memory>
#include <iostream>
#include <fstream>
#include <source_location>
#include <string_view>
#include <unordered_map>

// Create foxglove::schema::x objects
FOXGLOVE_SCHEMA(Pose)
FOXGLOVE_SCHEMA(Log)
FOXGLOVE_SCHEMA(PointCloud)

namespace logger {
/** Size of all particles and their weights + a little bit extra. */
static constexpr size_t BUILDER_SIZE =
    Localization::N * (sizeof(Localization::P) + sizeof(float)) + 64;

/** @brief Function to read, increment, and update a value in a file */
int incrementValueInFile(const std::string& filename) {
  // Step 1: Open the file for reading
  std::ifstream inFile(filename);
  if (!inFile) {
    std::cerr << "Error opening file for reading!" << std::endl;
    return -1; // Return -1 to indicate an error
  }

  // Step 2: Read the value from the file
  std::int64_t value;
  inFile >> value;
  inFile.close();

  // Step 3: Increment the value
  value++;

  // Open the file for writing
  std::ofstream outFile(filename);
  if (!outFile) {
    std::cerr << "Error opening file for writing!" << std::endl;
    return -1; // Return -1 to indicate an error
  }

  // Write the incremented value back to the file
  outFile << value << std::endl;
  outFile.close();

  return value;
}

static std::string getLogFilename() {
  constexpr const char* indexFilename = "/usd/localization.index";
  std::int64_t index = incrementValueInFile(indexFilename);
  std::string logFileNameStr =
      "/usd/localization." + std::to_string(index) + ".mcap";
  return logFileNameStr;
}

static mcap::McapWriterOptions makeMcapWriterOpts() {
  mcap::McapWriterOptions opts {""};
  opts.compression = mcap::Compression::Lz4;
  opts.compressionLevel = mcap::CompressionLevel::Fastest;

  opts.noRepeatedSchemas = true;
  opts.noRepeatedChannels = true;

  opts.noChunking = true;
  return opts;
};

extern "C" uint32_t ulPortSetInterruptMask(void);
extern "C" void vPortClearInterruptMask(uint32_t ulNewMaskValue);

#define DISABLE_INTERRUPTS() ulPortSetInterruptMask()
#define ENABLE_INTERRUPTS() vPortClearInterruptMask(0)

// Default:
// writer.write:   4.3751±1.9577
// LL.update():    5.9530±1.6812
// PF.predict:     0.5551±0.0058
// PF.update:      0.9661±0.0367
// PF.updateWeights:       0.9335±0.0256
// PF.printParticles:      4.3863±1.6811

// Fast:
// writer.write:   5.6750±8.5673
// LL.update():    7.2574±8.5220
// PF.predict:     0.5556±0.0070
// PF.update:      0.9671±0.0356
// PF.updateWeights:       0.9346±0.0265
// PF.printParticles:      5.6889±8.5220

// Fastest:
//  [ProfSummary.Start]
//  writer.write:   4.2929±5.8124
//  LL.update():    5.8780±5.7830
//  PF.predict:     0.5557±0.0095
//  PF.update:      0.9807±0.0400
//  PF.updateWeights:       0.9479±0.0299
//  PF.printParticles:      4.2959±5.7833
//  [ProfSummary.End]

// Slowest:
//  writer.write:   3.7698±2.1918
//  LL.update():    5.3435±1.9922
//  PF.predict:     0.5554±0.0066
//  PF.update:      0.9793±0.0356
//  PF.updateWeights:       0.9465±0.0241
//  PF.printParticles:      3.7631±1.9919

// Slow:
//  writer.write:   3.7260±1.7334
//  LL.update():    5.3115±1.6116
//  PF.predict:     0.5556±0.0076
//  PF.update:      0.9823±0.0348
//  PF.updateWeights:       0.9495±0.0248
//  PF.printParticles:      3.7280±1.6103

static mcap::Timestamp currentMcapTime() { return pros::micros() * 1000; }

static foxglove::Time makeFoxTime(mcap::Timestamp time) {
  return {static_cast<uint32_t>(time / 1000 / 1000 / 1000),
          static_cast<uint32_t>(time % (1000 * 1000 * 1000))};
}

struct LogTools {
    flatbuffers::FlatBufferBuilder builder;
    mcap::McapWriter writer;
  private:
    std::unordered_map<std::string_view, mcap::ChannelId> channels;
    static std::unique_ptr<pros::MutexVar<LogTools>> instance;

    friend pros::MutexVar<LogTools>;
    LogTools();
  public:
    /**
     * @brief Attempts to find the existing channel id for a topic.
     * If it is not found, callback is called to create it.
     */
    mcap::ChannelId getChannelId(const std::string_view topic,
                                 mcap::Schema& schema) {
      if (!channels.contains(topic))
        channels[topic] = [&] {
          writer.addSchema(schema);
          mcap::Channel channel(topic, "flatbuffer", schema.id);
          writer.addChannel(channel);
          return channel.id;
        }();
      return channels.at(topic);
    }

    inline static pros::MutexVarLock<LogTools> get() {
      if (instance == nullptr) {
        instance = std::make_unique<pros::MutexVar<LogTools>>();
      }
      return instance->lock();
    };
};

std::unique_ptr<pros::MutexVar<LogTools>> LogTools::instance = nullptr;

LogTools::LogTools() : builder(BUILDER_SIZE), channels {}, writer {} {
  const std::string logFilename = getLogFilename();
  const auto res = writer.open(logFilename, makeMcapWriterOpts());
  if (!res.ok()) {
    std::cerr << "Failed to open " << logFilename
              << " for writing: " << res.message << std::endl;
  }

  std::cout << "Opened " << logFilename << " for writing!" << std::endl;
};

void recordPose(lemlib::Pose pose, const std::string_view channelTopic) {
  auto lt = LogTools::get();
  mcap::ChannelId channelId =
      lt->getChannelId(channelTopic, foxglove::schemas::Pose);

  auto vector3 = foxglove::CreateVector3(lt->builder, pose.x, pose.y, 0);
  auto quaternion = foxglove::CreateQuaternion(
      lt->builder, 0, 0, sin(pose.theta / 2), cos(pose.theta / 2));
  auto foxPose = foxglove::CreatePose(lt->builder, vector3, quaternion);

  lt->builder.Finish(foxPose);

  mcap::Message msg {.channelId = channelId,
                     .logTime = currentMcapTime(),
                     .publishTime = currentMcapTime(),
                     .dataSize = lt->builder.GetSize(),
                     .data =
                         (const std::byte*)(lt->builder.GetBufferPointer())};
  const auto res = lt->writer.write(msg);
  if (!res.ok()) {
    std::cerr << "Failed to write message: " << res.message << "\n";
    lt->writer.terminate();
  }

  lt->builder.Clear();
};

void log(const std::string_view message, const std::string_view channelTopic,
         foxglove::LogLevel logLevel, std::source_location sourceLoc) {
  auto lt = LogTools::get();
  mcap::ChannelId channelId =
      lt->getChannelId(channelTopic, foxglove::schemas::Log);

  mcap::Timestamp time = currentMcapTime();
  const auto foxTime = makeFoxTime(time);

  const auto msgOffset = lt->builder.CreateString(message);
  const auto functionName = lt->builder.CreateString(sourceLoc.function_name());
  const auto filename = lt->builder.CreateString(sourceLoc.file_name());
  const auto logOffset =
      foxglove::CreateLog(lt->builder, &foxTime, logLevel, msgOffset,
                          functionName, filename, sourceLoc.line());

  lt->builder.Finish(logOffset);

  mcap::Message msg {.channelId = channelId,
                     .logTime = time,
                     .publishTime = time,
                     .dataSize = lt->builder.GetSize(),
                     .data =
                         (const std::byte*)(lt->builder.GetBufferPointer())};
  const auto res = lt->writer.write(msg);
  if (!res.ok()) {
    std::cerr << "Failed to write message: " << res.message << "\n";
    lt->writer.terminate();
  }

  lt->builder.Clear();
}

void recordParticles(Localization::PF::Data& data,
                     const std::string_view channelTopic) {
  // DISABLE_INTERRUPTS();
  auto lt = LogTools::get();
  mcap::ChannelId channelId =
      lt->getChannelId(channelTopic, foxglove::schemas::PointCloud);

  constexpr size_t FIELDS_PER_POINT = 3;

  mcap::Timestamp time = currentMcapTime();
  const auto foxTime = makeFoxTime(time);

  const auto fieldVector = [&] {
    std::array<flatbuffers::Offset<foxglove::PackedElementField>,
               FIELDS_PER_POINT>
        fields;
    // Describe the data layout to the consumer of the pointcloud. There are
    // 2 single-precision float fields describing the position of each
    // particle.
    const char* const fieldNames[] = {"x", "y", "z"};
    static_assert(sizeof(fieldNames) / sizeof(fieldNames[0]) ==
                      FIELDS_PER_POINT,
                  "field arrays do not have equal length");

    int fieldOffset = 0;
    for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
      auto fieldName = lt->builder.CreateString(fieldNames[i]);
      auto field = foxglove::CreatePackedElementField(
          lt->builder, fieldName, fieldOffset, foxglove::NumericType_FLOAT32);
      fields[i] = field;
      fieldOffset += sizeof(Localization::P::Scalar);
    }

    return lt->builder.CreateVector(fields.begin(), fields.size());
  }();

  static std::array<float, FIELDS_PER_POINT * Localization::N>* dataArr =
      new std::array<float, FIELDS_PER_POINT * Localization::N>();
  for (size_t i = 0; i < Localization::N; ++i) {
    (*dataArr)[3 * i] = (*data.particles)[i][0];
    (*dataArr)[3 * i + 1] = (*data.particles)[i][1];
    (*dataArr)[3 * i + 2] = (*data.weights)[i];
  }

  const auto dataVectorOffset = lt->builder.CreateVector(
      reinterpret_cast<const uint8_t*>(dataArr->data()), sizeof(*dataArr));

  const auto pose = foxglove::CreatePose(
      lt->builder, foxglove::CreateVector3(lt->builder, 0, 0, 0),
      foxglove::CreateQuaternion(lt->builder, 0, 0, 0, 1));
  /** @brief Frame of reference */
  const auto frameId = lt->builder.CreateString(channelTopic);

  const auto pointCloud = foxglove::CreatePointCloud(
      lt->builder, &foxTime, frameId, pose, sizeof(float) * FIELDS_PER_POINT,
      fieldVector, dataVectorOffset);
  lt->builder.Finish(pointCloud);

  mcap::Message msg {.channelId = channelId,
                     .logTime = time,
                     .publishTime = time,
                     .dataSize = lt->builder.GetSize(),
                     .data =
                         (const std::byte*)(lt->builder.GetBufferPointer())};
  PROFILE_SECTION("writer.write", {
    const auto res = lt->writer.write(msg);
    if (!res.ok()) {
      std::cerr << "Failed to write message: " << res.message << "\n";
      lt->writer.terminate();
    }
  })

  // {
  //   auto* sink = lt->writer.dataSink();
  //   if (sink != nullptr) PROFILE_SECTION("writer.flush", { sink->flush();
  //   })
  // }

  lt->builder.Clear();

  // ENABLE_INTERRUPTS();
}
}; // namespace logger