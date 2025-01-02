/**
 * @brief Times sections of code (Poor man's profiling).
  Describes the actual implementation of the profiler.
 */
#pragma once

#ifndef __PROFILER__INCLUDED_FROM_API_H__
#error "Do not include this file directly. Include profiler/api.h instead."
#endif

#include "units/units.hpp"
#include "pros/rtos.hpp"
#include "pros/rtos.hpp"
#include "units/units.hpp"
#include <cstddef>
#include <memory>

/** @brief Wraps pros::micros() in a Time Quantity */
inline Time getNow() { return double(pros::micros()) * usec; }

/** @brief Provides api for timing a single section of code */
class SectionProfiler {
  private:
    /** @brief Number of times the profiled code section has been timed */
    size_t m_N = 0;
    /** @brief Name for instrumented code */
    char* m_name;

    /** @brief Mean of duration of instrumented code */
    Time m_muDuration = 0 * sec;
    /** @brief Mean of the square of the duration of instrumented code (used to
     * find std deviation) */
    decltype(sec * sec) m_muDuration2 = 0 * sec * sec;

    /** @param name Name for instrumented code */
    SectionProfiler(char* name) : m_name(name) {};
  public:
    /** @brief Provides a simple api to get time since its construction.  */
    struct Stopwatch {
        const Time start = getNow();

        /** @returns time since construction in micros */
        inline Time measure() const { return getNow() - start; };
    };

    /** @brief Used to communicate distribution of timings to Profiler. */
    struct Report {
        /** Name for instrumented code */
        char* name;
        /** Average duration of instrumented code */
        Time mu;
        /** Standard deviation of duration of instrumented code */
        Time sigma;
    };

    /** @brief Starts the stopwatch */
    std::unique_ptr<Stopwatch> start() const;

    /** @brief Ends the stopwatch and records the measured duration. */
    void end(std::unique_ptr<Stopwatch> stopwatch);

    /** @brief Generates report of timing distribution. */
    Report report() const;

    /** @param name Name for instrumented code */
    static std::shared_ptr<SectionProfiler> make(char* name);
};

/** @brief Singleton for printing timing reports from all sections. */
class Profiler {
  private:
    /** @brief Singleton instance of self */
    static Profiler* self;

    /** @brief All sections ever created */
    std::vector<std::shared_ptr<SectionProfiler>> m_sections;
  public:
    /**
     * @brief Prints out the report of all the profilers.
     *
     * @param printHeader True if the "[Profiler]: <time>\n" header should be
     * printed before the report
     * @param prefix String to be printed before each line of the report (does
     * not include header)
     */
    void printReport(bool printHeader = true, char* prefix = (char*)"\t");

    /** @brief Adds a section to the list of sections */
    void addSection(std::shared_ptr<SectionProfiler> s);

    /** @returns A singleton instance of Profiler */
    static inline Profiler& get() {
      if (self == nullptr) self = new Profiler;
      return *self;
    }
};

/**
 * @brief Below are macros that make it easier to profile sections of code.
 */
#ifndef _NO_PROFILE_MACROS

/**
 * @brief Starts a section of profiled code with a given label.
 * @param label Name of the profiled section.
 * @param var_prefix Prefix for the profiling variables created by the macro in
 * order to prevent name conflicts.
 */
#define START_PREFIXED_PROFILE(label, var_prefix)                              \
  static auto var_prefix##_profiler = SectionProfiler::make((char*)label);     \
  auto var_prefix##_stopwatch = var_prefix##_profiler->start();

/**
 * @brief Ends section of profiled code.
 * @param var_prefix Prefix for the profiling variables in order to prevent name
 * conflicts.
 */
#define END_PREFIXED_PROFILE(var_prefix)                                       \
  var_prefix##_profiler->end(std::move(var_prefix##_stopwatch));

/**
 * @brief Starts a section of profiled code with a given label.
 * @param label Name of the profiled section.
 * @warning If variable name conflicts occur, use START_PREFIXED_PROFILE.
 */
#define START_PROFILE(label) START_PREFIXED_PROFILE(label, __prof)
/**
 * @brief Ends section of profiled code.
 * @warning If variable name conflicts occur, use END_PREFIXED_PROFILE.
 */
#define END_PROFILE() END_PREFIXED_PROFILE(__prof)

/**
 * @brief Profiles a block of code with the given label.
 *
 * @param label Name of the profiled section.
 * @param code Code to profile.
 *
 * @note Variables defined inside of code will not be accessible outside of the
 * block.
 *
 * @b Example
 * @code {.cpp}
 * PROFILE_SECTION("PF.update", { m_pf.update(m_sensors, m_resampler); })
 * @endcode
 *
 * @b Example - A full function
 * @code {.cpp}
 * double integral_of_sin_0_to_2pi() PROFILE_SECTION(__func__, {
 *    // Computes the integral using a left Riemann sum
 *    double min = 0;
 *    double max = 2 * M_PI;
 *    double deltaX = (max - min) / 1000.f;
 *    double sum = 0;
 *    for (double x = min; x < max; x += deltaX) {
 *      sum += deltaX;
 *    }
 *    return sum;
 * })
 * @endcode
 */
#define PROFILE_SECTION(label, code...)                                        \
  {                                                                            \
    START_PROFILE(label)                                                       \
    code;                                                                      \
    END_PROFILE()                                                              \
  }

#endif