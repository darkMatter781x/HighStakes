#include "pf/util/lines.h"
#include <cmath>
#include <optional>
#include <vector>

#include "Eigen/Dense"

namespace lines {

Segment Segment::fromEndPts(Eigen::Vector2f start, Eigen::Vector2f end) {
  return {.start = start, .vec = end - start};
}

std::vector<Segment> Segment::fromPts(std::vector<Eigen::Vector2f> pts) {
  std::vector<Segment> segments;
  for (size_t i = 0; i < pts.size() - 1; i++) {
    segments.push_back(Segment::fromEndPts(pts[i], pts[i + 1]));
  }
  return segments;
}

// TODO: test
std::optional<float> Ray::distanceTo(const Segment& other) const {
  // Source: https://stackoverflow.com/a/565282

  auto& q = other.start;
  auto& s = other.vec;
  auto& p = start;
  auto& r = direction;

  const float r_x_s = r.cross(s);
  // Ray and segment are parallel / colinear
  if (r_x_s == 0) return std::nullopt;

  const auto q_minus_p = q - p;

  // u = ((q - p) x r) / (r x s)
  const float u = (q_minus_p.cross(r)) / r_x_s;

  // Ray doesn't hit segment
  if (u < 0 || u > 1) return std::nullopt;

  // t = ((q - p) x s) / (r x s)
  // Represents distance
  const float t = (q_minus_p.cross(s)) / r_x_s;

  // Ray facing away from segment
  if (t < 0) return std::nullopt;

  return t;
}

std::optional<std::pair<float, size_t>>
Ray::findFirstIntersect(const std::vector<Segment>& segments) const {
  // Smallest distance found
  float minDist = INFINITY;
  // Index corresponding to smallest distance found
  size_t index = -1;

  for (size_t i = 0; i < segments.size(); i++) {
    // Calculate distance
    auto distance = this->distanceTo(segments[i]);

    // Handle std::null_opt
    if (!distance.has_value()) continue;
    // Isn't smallest dist found
    if (distance.value() > minDist) continue;

    minDist = distance.value();
    index = i;
  }
  // Didn't intersect with anything
  if (!std::isfinite(minDist)) return std::nullopt;

  return std::make_optional<std::pair<float, size_t>>({minDist, index});
}
} // namespace lines