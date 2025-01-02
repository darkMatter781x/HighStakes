/**
 * @brief Code profiling utilities.
 * The actual code is hidden in ./impl.hpp to ensure profiling can be disabled
 * by ENABLE_PROFILING whilst still being provided intellisense when looking at
 * the header.
 */

#pragma once

// Only profile code if ENABLE_PROFILING is defined. This lets us easily disable
// profiling for production code.
#if defined(ENABLE_PROFILING)
#define __PROFILER__INCLUDED_FROM_API_H__
#include "profiler/_impl.h"
#else
// Placeholder macros for when profiling is disabled.
#define START_PREFIXED_PROFILE(label, var_prefix)
#define END_PREFIXED_PROFILE(var_prefix)
#define START_PROFILE(label)
#define END_PROFILE()
#define PROFILE_SECTION(label, code...)                                        \
  { code; }

#endif