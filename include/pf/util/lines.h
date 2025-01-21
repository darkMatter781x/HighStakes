#pragma once

#include "Eigen/Core"
#include <optional>

// Used for distance sensor expected measurements

namespace lines {

struct Segment {
    /** Start point */
    Eigen::Vector2f start;
    /**
     * @brief Represents the vector from start that creates the end points
     * end = start + vec
     */
    Eigen::Vector2f vec;

    static Segment fromEndPts(Eigen::Vector2f start, Eigen::Vector2f end);

    /**
     * @brief Connects each point to the next with a line segment, returning a
     * vector of segments.
     * @param pts Vector of points.
     * @returns Vector of segments.
     */
    static std::vector<Segment> fromPts(std::vector<Eigen::Vector2f> pts);
};

struct Ray {
    Eigen::Vector2f start;
    /** Must have a length of 1 */
    Eigen::Vector2f direction;

    /**
     * @brief Finds the distance to line segment
     *
     * @warning If the lines are colinear, returns std::nullopt.
     *
     * @returns Distance to line segment if the lines intersect.
     * @returns std::nullopt if the lines do not intersect.
     */
    std::optional<float> distanceTo(const Segment& other) const;

    /**
     * @brief Finds the distance to the nearest line segment.
     *
     * @returns std::pair<float, int>
     * first: Distance to line segment if the lines intersect.
     * second: Index of the line segment in the vector.
     *
     * @returns std::nullopt If none of the line segments intersect with this
     * ray.
     */
    std::optional<std::pair<float, size_t>>
    findFirstIntersect(const std::vector<Segment>& segments) const;
};
} // namespace lines