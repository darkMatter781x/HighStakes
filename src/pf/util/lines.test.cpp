#ifdef GTEST

#include <vector>
#include <gtest/gtest.h>
#include "pf/util/lines.h"

// Demonstrate some basic assertions.
TEST(Lines, Segment_fromEndPts) {
  auto seg = lines::Segment::fromEndPts({5, -3}, {1, 7});
  EXPECT_FLOAT_EQ(seg.start.x(), 5);
  EXPECT_FLOAT_EQ(seg.start.y(), -3);
  EXPECT_FLOAT_EQ(seg.vec.x(), -4);
  EXPECT_FLOAT_EQ(seg.vec.y(), 10);
}

TEST(Lines, Segment_fromPts) {
  auto segments = lines::Segment::fromPts({{5, -3}, {1, 7}, {2, 11}});
  EXPECT_EQ(segments.size(), 2);
  auto seg1 = segments[0];
  auto seg2 = segments[1];
  EXPECT_FLOAT_EQ(seg1.start.x(), 5);
  EXPECT_FLOAT_EQ(seg1.start.y(), -3);
  EXPECT_FLOAT_EQ(seg1.vec.x(), -4);
  EXPECT_FLOAT_EQ(seg1.vec.y(), 10);

  EXPECT_FLOAT_EQ(seg2.start.x(), 1);
  EXPECT_FLOAT_EQ(seg2.start.y(), 7);
  EXPECT_FLOAT_EQ(seg2.vec.x(), 1);
  EXPECT_FLOAT_EQ(seg2.vec.y(), 4);
}

TEST(Lines, Ray_distanceTo_Hit) {
  auto segment = lines::Segment {.start = {5, -3}, .vec = {-12, 4}};
  auto ray =
      lines::Ray {.start = {3, -1}, .direction = {sqrt(2) / 2, -sqrt(2) / 2}};
  auto dist = ray.distanceTo(segment);
  EXPECT_TRUE(dist.has_value());
  EXPECT_FLOAT_EQ(dist.value(), 2 * sqrt(2));
}

TEST(Lines, Ray_distanceTo_MissFacingAway) {
  auto segment = lines::Segment {.start = {5, -3}, .vec = {-12, 4}};
  auto ray =
      lines::Ray {.start = {3, -1}, .direction = {sqrt(2) / 2, sqrt(2) / 2}};
  auto dist = ray.distanceTo(segment);
  EXPECT_FALSE(dist.has_value());
}

TEST(Lines, Ray_distanceTo_MissParallel) {
  auto segment = lines::Segment {.start = {5, -3}, .vec = {-12, 4}};
  auto ray =
      lines::Ray {.start = {3, -1}, .direction = segment.vec.normalized()};
  auto dist = ray.distanceTo(segment);
  EXPECT_FALSE(dist.has_value());
}

TEST(Lines, Ray_distanceTo_HitColinear) {
  auto segment = lines::Segment {.start = {5, -3}, .vec = {-12, 4}};
  auto ray =
      lines::Ray {.start = {3, -1}, .direction = segment.vec.normalized()};
  auto dist = ray.distanceTo(segment);
  EXPECT_TRUE(dist.has_value());
  EXPECT_FLOAT_EQ(dist.value(), 0);
}

TEST(Lines, Ray_distanceTo_MissBarely) {
  auto segment = lines::Segment {.start = {5, -3}, .vec = {-12, 4}};
  auto ray = lines::Ray {.start = {3, -1},
                         .direction = Eigen::Vector2f {1, -.9}.normalized()};
  auto dist = ray.distanceTo(segment);
  EXPECT_FALSE(dist.has_value());
}

TEST(Lines, Ray_findFirstIntersect_Miss) {
  auto segments =
      std::vector<lines::Segment> {{.start = {5, -3}, .vec = {-12, 4}}};
  auto ray =
      lines::Ray {.start = {3, -1}, .direction = {sqrt(2) / 2, sqrt(2) / 2}};
  auto intersect = ray.findFirstIntersect(segments);
  EXPECT_FALSE(intersect.has_value());
}

TEST(Lines, Ray_findFirstIntersect_MissEmptyVector) {
  auto segments = std::vector<lines::Segment> {};
  auto ray =
      lines::Ray {.start = {3, -1}, .direction = {sqrt(2) / 2, sqrt(2) / 2}};
  auto intersect = ray.findFirstIntersect(segments);
  EXPECT_FALSE(intersect.has_value());
}

#endif