#pragma once

#include "color.h"
#include "lemlib/chassis/chassis.hpp"
#include <memory>

namespace auton {

/**
 * @brief Abstract interface for transformations on motion parameters.
 * Used to enable mirroring of autonomous routines.
 */
class Transformation {
  public:
    /** @note Assumes theta is in compass degrees. */
    virtual lemlib::Pose transformPose(lemlib::Pose oldPose) = 0;

    virtual lemlib::DriveSide
    transformDriveSide(lemlib::DriveSide oldDriveSide) = 0;

    virtual lemlib::AngularDirection
    transformAngDir(lemlib::AngularDirection oldAngDir) = 0;

    /** @note Assumes theta is in compass degrees. */
    virtual float transformHeading(float oldHeading) = 0;
};

class ChainedTransform : public Transformation {
  private:
    /** @brief First transform */
    std::shared_ptr<Transformation> m_t1;
    /** @brief Second transform */
    std::shared_ptr<Transformation> m_t2;
  public:
    /** @note Assumes theta is in compass degrees. */
    virtual lemlib::Pose transformPose(lemlib::Pose oldPose) override;

    virtual lemlib::DriveSide
    transformDriveSide(lemlib::DriveSide oldDriveSide) override;

    virtual lemlib::AngularDirection
    transformAngDir(lemlib::AngularDirection oldAngDir) override;

    /** @note Assumes theta is in compass degrees. */
    virtual float transformHeading(float oldHeading) override;

    ChainedTransform(std::shared_ptr<Transformation> transformA,
                     std::shared_ptr<Transformation> transformB);

    /**
     * @brief Creates a chained transform by transforming the output from
     * transformA with transformB.
     *
     * input ============> ============> output
     *        transformA    transformB
     */
    static std::shared_ptr<ChainedTransform>
    make(std::shared_ptr<Transformation> transformA,
         std::shared_ptr<Transformation> transformB);
};

/**
 * @brief Mirrors the robot's position and heading for red / blue alliances.
 * ( Mirrors over the y-axis ).
 */
class AllianceTransform : public Transformation {
  public:
    /** @brief The alliance the auto was programmed for. */
    const ALLIANCE normalAlliance;

    /** @brief The alliance currently running. */
    ALLIANCE alliance;

    bool needsTransform() const;

    /** @note Assumes theta is in compass degrees. */
    virtual lemlib::Pose transformPose(lemlib::Pose oldPose) override;

    virtual lemlib::DriveSide
    transformDriveSide(lemlib::DriveSide oldDriveSide) override;

    virtual lemlib::AngularDirection
    transformAngDir(lemlib::AngularDirection oldAngDir) override;

    /** @note Assumes theta is in compass degrees. */
    virtual float transformHeading(float oldHeading) override;

    /**
     * @brief Construct a new AllianceTransform object
     *
     * @param alliance The alliance currently running.
     * @param normalAlliance The alliance the auto was programmed for.
     */
    AllianceTransform(ALLIANCE alliance,
                      ALLIANCE normalAlliance = ALLIANCE::RED);
};

/**
 * @brief Mirrors the robot's position and heading for positive / negative
 * sides. ( Mirrors over the x-axis ).
 */
class SignTransform : public Transformation {
  public:
    /** @brief Refers to the sign of the corners on that side of the field. */
    enum SIGN {
      POSITIVE,
      NEGATIVE,
    };

    /** @brief The sign that the auto was programmed for. */
    const SIGN normalSign;

    /** @brief The sign thats being currently run. */
    SIGN sign;

    bool needsTransform() const;

    /** @note Assumes theta is in compass degrees. */
    virtual lemlib::Pose transformPose(lemlib::Pose oldPose) override;

    virtual lemlib::DriveSide
    transformDriveSide(lemlib::DriveSide oldDriveSide) override;

    virtual lemlib::AngularDirection
    transformAngDir(lemlib::AngularDirection oldAngDir) override;

    /** @note Assumes theta is in compass degrees. */
    virtual float transformHeading(float oldHeading) override;

    /**
     * @brief Construct a new SignTransform object
     *
     * @param sign The sign thats being currently run.
     * @param normalSign The sign that the auto was programmed for.
     */
    SignTransform(SIGN sign, SIGN normalSign);
};
} // namespace auton