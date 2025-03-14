#include "pf/util/logger.h"

#include "foxglove/Log_generated.h"
#include "foxglove/PointCloud_generated.h"
#include "foxglove/PosesInFrame_generated.h"
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
#include <queue>
#include <source_location>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

// Create foxglove::schema::x objects
FOXGLOVE_SCHEMA(PosesInFrame)
FOXGLOVE_SCHEMA(Log)
FOXGLOVE_SCHEMA(PointCloud)

namespace logger {
/** Size of all particles and their weights + a little bit extra. */
static constexpr size_t BUILDER_SIZE =
    Localization::N * (sizeof(Localization::P) + sizeof(float)) + 64;

/** @brief Function to read, increment, and update a value in a file */
static int incrementValueInFile(const std::string& filename) {
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
  opts.compression = mcap::Compression::None;
  // opts.compressionLevel = mcap::CompressionLevel::Fastest;

  opts.noRepeatedSchemas = true;
  opts.noRepeatedChannels = true;

  opts.noChunking = true;
  return opts;
};

extern "C" uint32_t ulPortSetInterruptMask(void);
extern "C" void vPortClearInterruptMask(uint32_t ulNewMaskValue);

#define DISABLE_INTERRUPTS() ulPortSetInterruptMask()
#define ENABLE_INTERRUPTS() vPortClearInterruptMask(0)

static mcap::Timestamp currentMcapTime() { return pros::micros() * 1000; }

static foxglove::Time makeFoxTime(mcap::Timestamp time) {
  return {static_cast<uint32_t>(time / 1000 / 1000 / 1000),
          static_cast<uint32_t>(time % (1000 * 1000 * 1000))};
}

/**
 * @brief Represents a queued message. Owns its data pointer.
 * @see QueuedMcapWriter
 */
struct QueuedMessage {
    mcap::Message msg;
    std::vector<std::byte> data;

    static std::unique_ptr<QueuedMessage> fromMcap(mcap::Message mMsg) {
      auto qMsg = std::make_unique<QueuedMessage>(
          mMsg, std::vector<std::byte> {mMsg.data, mMsg.data + mMsg.dataSize});
      qMsg->msg.data = qMsg->data.data();
      return qMsg;
    }
};

/**
 * @brief Queues messages sent to it, and writes them in its own task
 */
class QueuedMcapWriter {
    pros::Task m_task;
    pros::MutexVar<mcap::McapWriter> m_writerMutex;
    pros::MutexVar<std::queue<std::unique_ptr<QueuedMessage>>> m_queueMutex;

    /** @returns Number of mcap messages that are queued */
    size_t size() { return m_queueMutex.lock()->size(); }

    /** @brief Removes the first element and returns it */
    std::unique_ptr<QueuedMessage> popFront() {
      auto queue = m_queueMutex.lock();

      auto res = std::move(queue->front());
      queue->pop();
      return res;
    }

    void loop() {
      while (true) {
        if (this->size() == 0) {
          // Clear notifs
          pros::Task::current().notify_clear();
          // Wait until notification received (message placed in buffer)
          pros::Task::notify_take(true, TIMEOUT_MAX);
        }
        if (this->size() == 0) continue;

        std::unique_ptr<QueuedMessage> msg = this->popFront();

        {
          PROFILE_SECTION("writer.write()", {
            auto writer = this->getWriter();
            const auto res = writer->write(msg->msg);
            if (!res.ok()) {
              static std::string prevErrMsg = "";
              if (res.message != prevErrMsg)
                fprintf(stderr, "[%2.6f]:\tfailed to log to sd card!:\n\t%s\n",
                        float(pros::micros()) / 1000.f / 1000.f,
                        res.message.c_str());
              prevErrMsg = res.message;
              writer->terminate();
            }
          })
        }
      }
    }
    friend class LogTools;

    QueuedMcapWriter()
      : m_writerMutex(), m_task([&] { this->loop(); }, TASK_PRIORITY_MIN) {
      const std::string logFilename = getLogFilename();
      const auto res =
          this->getWriter()->open(logFilename, makeMcapWriterOpts());

      std::cout << "Opened " << logFilename << " for writing!" << std::endl;
    }
  public:
    /** @brief Locks and returns the underlying mcap writer */
    pros::MutexVarLock<mcap::McapWriter> getWriter() {
      return m_writerMutex.lock();
    }

    void write(const mcap::Message& message) {
      // printf("[%2.6f]:\tadd message to queue!\n",
      //        float(pros::micros()) / 1000.f / 1000.f);
      m_queueMutex.lock()->push(QueuedMessage::fromMcap(message));
      m_task.notify();
    }
};

struct LogTools {
    flatbuffers::FlatBufferBuilder builder;
    QueuedMcapWriter writer;
    std::unordered_map<std::string_view, mcap::ChannelId> m_channels;
  private:
    static std::unique_ptr<pros::MutexVar<LogTools>> instance;

    friend pros::MutexVar<LogTools>;
    LogTools();
  public:
    inline static pros::MutexVarLock<LogTools> get() {
      if (instance == nullptr) {
        printf("making LogTools instance!\n");
        instance = std::make_unique<pros::MutexVar<LogTools>>();
      }
      return instance->lock();
    };

    /**
     * @brief Attempts to find the existing channel id for a topic.
     * If it is not found, callback is called to create it.
     */
    mcap::ChannelId getChannelId(const std::string_view topic,
                                 mcap::Schema& schema) {
      auto w = this->writer.getWriter();
      if (!m_channels.contains(topic))
        m_channels[topic] = [&] {
          w->addSchema(schema);
          mcap::Channel channel(topic, "flatbuffer", schema.id);
          w->addChannel(channel);
          return channel.id;
        }();
      return m_channels.at(topic);
    }
};

std::unique_ptr<pros::MutexVar<LogTools>> LogTools::instance = nullptr;

LogTools::LogTools() : builder(BUILDER_SIZE), writer {} {};

void recordPose(lemlib::Pose pose, const std::string_view channelTopic,
                const std::string_view frameId) {
  auto lt = LogTools::get();
  mcap::ChannelId channelId =
      lt->getChannelId(channelTopic, foxglove::schemas::PosesInFrame);

  const mcap::Timestamp mcapTime = currentMcapTime();
  const foxglove::Time foxTime = makeFoxTime(mcapTime);

  /** @brief Frame of reference */
  const auto frameIdOffset = lt->builder.CreateString(frameId);

  const auto vector3 = foxglove::CreateVector3(lt->builder, pose.x, pose.y, 0);
  const auto quaternion = foxglove::CreateQuaternion(
      lt->builder, 0, 0, sin(pose.theta / 2), cos(pose.theta / 2));
  const auto foxPose = foxglove::CreatePose(lt->builder, vector3, quaternion);
  const auto poseVector = lt->builder.CreateVector({foxPose});

  const auto posesInFrame = foxglove::CreatePosesInFrame(
      lt->builder, &foxTime, frameIdOffset, poseVector);
  lt->builder.Finish(posesInFrame);

  mcap::Message msg {.channelId = channelId,
                     .logTime = mcapTime,
                     .publishTime = mcapTime,
                     .dataSize = lt->builder.GetSize(),
                     .data =
                         (const std::byte*)(lt->builder.GetBufferPointer())};

  lt->writer.write(msg);
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
  lt->writer.write(msg);
  lt->builder.Clear();
}

void recordParticles(Localization::PF::Data& data,
                     const std::string_view channelTopic,
                     const std::string_view frameId) {
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
  const auto frameIdOffset = lt->builder.CreateString(frameId);

  const auto pointCloud = foxglove::CreatePointCloud(
      lt->builder, &foxTime, frameIdOffset, pose,
      sizeof(float) * FIELDS_PER_POINT, fieldVector, dataVectorOffset);
  lt->builder.Finish(pointCloud);

  mcap::Message msg {.channelId = channelId,
                     .logTime = time,
                     .publishTime = time,
                     .dataSize = lt->builder.GetSize(),
                     .data =
                         (const std::byte*)(lt->builder.GetBufferPointer())};
  lt->writer.write(msg);
  lt->builder.Clear();
}
}; // namespace logger