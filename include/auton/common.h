#pragma once

#include "color.h"
#include "auton/transform.h"

/**
 * @brief First corner of AWP.
 * Assumes that the bot starts such that it can score on alliance stake
 * immediately. The quadrant the bot starts in should correspond with the
 * alliance and corner params.
 * Scores preload on ally stake, and scores the alliance stack ring and the safe
 * ring on the mogo originiting from the quadrant the function was started on.
 * Score Summary:
 * - 1 on alliance stake
 * - 1 or 2 on mogo (depending on grabAllyStack param)
 *
 * @warning Sets the bot position to the starting position for immediate scoring
 * on the wall stake.
 *
 * @warning Mutates localization transform!
 *
 * @param alliance Which alliance the robot is.
 * @param corner Which corner the robot starts on. This will be the side that
 * the auto scores rings on.
 * @param grabAllyStack Whether to grab the ring in front of the alliance wall
 * stake.
 */
void firstAWPCorner(ALLIANCE alliance, auton::SignTransform::SIGN corner,
                    bool grabAllyStack);