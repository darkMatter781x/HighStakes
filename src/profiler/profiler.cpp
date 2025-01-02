// TODO: Use a better macro than `__clang__` to enable profiling
/* If using intellisense/clangd, enable profiling so code is not hidden. */
#if defined(__INTELLISENSE__) or defined(__clang__)
#define ENABLE_PROFILING
#endif

#include "profiler/profiler.h"
#ifdef ENABLE_PROFILING

/**
 * @brief Calculates a running average. All elements are equally weighted
 *
 * @tparam T Type of data
 * @param mean Old average of (length - 1) data points
 * @param newDatum New data point to incorporate into mean
 * @param length Number of data points, including newDatum
 *
 * @returns New mean that incorporates new datum for an average of length pts
 */
template <typename T> T runningAverage(T mean, T newDatum, size_t length) {
  mean *= (length - 1) / float(length);
  mean += newDatum / float(length);
  return mean;
}

Profiler* Profiler::self = nullptr;

std::unique_ptr<SectionProfiler::Stopwatch> SectionProfiler::start() const {
  return std::make_unique<Stopwatch>();
};

SectionProfiler::Report SectionProfiler::report() const {
  return {.name = m_name,
          .mu = m_muDuration,
          .sigma = units::sqrt(m_muDuration2 - m_muDuration * m_muDuration)};
}

std::shared_ptr<SectionProfiler> SectionProfiler::make(char* name) {
  std::shared_ptr<SectionProfiler> s(new SectionProfiler(name));
  Profiler::get().addSection(s);
  return s;
}

void SectionProfiler::end(std::unique_ptr<Stopwatch> stopwatch) {
  const Time duration = stopwatch->measure();
  m_N++;
  m_muDuration = runningAverage(m_muDuration, duration, m_N);
  m_muDuration2 = runningAverage(m_muDuration2, duration * duration, m_N);
}

void Profiler::addSection(std::shared_ptr<SectionProfiler> section) {
  m_sections.push_back(section);
}

void Profiler::printReport(bool printHeader, char* prefix) {
  size_t i = 0;
  if (printHeader) printf("[Profiler]: %2.4fs\n", getNow().convert(sec));
  for (const auto& s : m_sections) {
    const auto report = s->report();

    if (++i % 2 == 0) printf("\x1b[2m");
    else printf("\x1b[22m");

    printf("%s%s:\t%2.4f±%2.4f\n", prefix, report.name, report.mu.convert(msec),
           report.sigma.convert(msec));
  }
  printf("\x1b[0m");
};

#endif