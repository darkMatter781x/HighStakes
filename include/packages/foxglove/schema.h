#pragma once

// TODO: move into foxglove dir (requires changing makefile)

#include "mcap/types.hpp" // IWYU pragma: export
#include <cstdint> // IWYU pragma: export

/**
 * @brief Creates a mcap::Schema object for a given foxglove schema.
 *
 * @param x The name of the schema.
 *
 * @example
 * ```cpp
 * FOXGLOVE_SCHEMA(PointCloud);
 * // ...
 * writer.addSchema(foxglove::schemas::PointCloud);
 * mcap::Channel channel("pointcloud", "flatbuffer",
 *  foxglove::schemas::PointCloud.id);
 * ```
 */

#define FOXGLOVE_SCHEMA(x)                                                     \
  extern "C" {                                                                 \
  extern uint8_t _binary_static_foxglove_##x##_bfbs_start[],                   \
      _binary_static_foxglove_##x##_bfbs_size[];                               \
  }                                                                            \
  static std::vector<std::byte> x##Bytes(                                      \
      (std::byte*)_binary_static_foxglove_##x##_bfbs_start,                    \
      (std::byte*)_binary_static_foxglove_##x##_bfbs_start +                   \
          (std::size_t)_binary_static_foxglove_##x##_bfbs_size);               \
  namespace foxglove::schemas {                                                \
  mcap::Schema x("foxglove." #x, "flatbuffer", x##Bytes);                      \
  }