#pragma once

enum class ALLIANCE { RED, BLUE };

using COLOR = ALLIANCE;

/** @brief Returns the opposite color. */
inline COLOR operator!(COLOR color) {
  switch (color) {
    case COLOR::RED: return COLOR::BLUE;
    case COLOR::BLUE: return COLOR::RED;
  }
}
