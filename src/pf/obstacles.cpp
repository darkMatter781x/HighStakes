#include "pf/obstacles.h"
#include "util/lines.h"
#include "dimensions.h"

std::vector<lines::Segment> getObstacles() {
  using namespace dimensions::field;

  return lines::Segment::fromPts({
      {MAX_X, MAX_Y},
      {MAX_X, MIN_Y},
      {MIN_X, MIN_Y},
      {MIN_X, MAX_Y},
      {MAX_X, MAX_Y},
  });
}