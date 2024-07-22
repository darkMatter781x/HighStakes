#pragma once

namespace dimensions {
namespace field {
namespace long_headings {
/** heading to referee */
constexpr float REFEREE = 0;
/** heading to face audience */
constexpr float AUDIENCE = 180;
/** heading to face red station */
constexpr float RED_STATION = 270;
/** heading to face blue station */
constexpr float BLUE_STATION = 90;
}; // namespace long_headings

using namespace long_headings;

namespace short_headings {
/** heading to face referee */
constexpr float REF = long_headings::REFEREE;
/** heading to face audience (stands) */
constexpr float STANDS = long_headings::AUDIENCE;
/** heading to face red driver station */
constexpr float RED = long_headings::RED_STATION;
/** heading to face blue driver station */
constexpr float BLUE = long_headings::BLUE_STATION;
} // namespace short_headings
namespace DIR = short_headings;

constexpr float TILE = 24;

/** x axis goes from red driver station to the blue driver station */
constexpr float MAX_X = TILE * 3;
constexpr float MIN_X = -MAX_X;
/** x axis goes from red driver station to the blue driver station */
constexpr float MAX_Y = TILE * 3;
constexpr float MIN_Y = -MAX_Y;

} // namespace field

namespace robot {
/** @todo change to correct track width */
constexpr float TRACK_WIDTH = 27.0 / 2;
constexpr float DRIVE_WIDTH = 29.0 / 2;
constexpr float DRIVE_LENGTH = 30.0 / 2;

} // namespace robot

namespace all {
using namespace field;
using namespace robot;
} // namespace all
} // namespace dimensions