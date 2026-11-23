#pragma once

namespace core
{

/// Input geometry for the slew calculation (pure, no Arduino deps).
struct SlewGeometry {
    float moveRA;                /// Target RA in hours, normalized to [-12, 12]
    float moveDEC;               /// Target DEC in degrees (already hemisphere-corrected)
    float homeTargetDeltaRA;     /// Delta between target RA and tracked home position (hours)
    float stepsPerSiderealHour;  /// u-steps per sidereal hour
    float stepsPerDECDegree;     /// u-steps per DEC degree
    float zeroPosDEC;            /// Accumulated DEC offset from sync (degrees)
    float raLimitL;              /// Lower RA limit (hours, negative)
    float raLimitR;              /// Upper RA limit (hours, positive)
};

/// Result of the slew calculation.
struct SlewResult {
    long targetRASteps;
    long targetDECSteps;

    /// Three candidate solutions (6 values: RA,DEC, RA,DEC, RA,DEC)
    long solutions[6];
};

/// Calculate RA and DEC stepper target positions given the geometry.
/// Implements the meridian-flip solution selector:
/// - Solution 1: direct (no flip)
/// - Solution 2: flip when past upper limit (moveRA -= 12, moveDEC = -moveDEC)
/// - Solution 3: flip when past lower limit (moveRA += 12, moveDEC = -moveDEC)
SlewResult calculateStepperPositions(const SlewGeometry &geo);

/// RA hours the sky has moved past a mount while its tracker was off.
/// completedOffHours: finished tracker-off time in (solar) hours. Can be slightly negative
///                    right after a post-slew catch-up has been subtracted from it.
/// currentOffMs:      length of the tracker-off period that is still running (0 while tracking).
float untrackedRaHours(float completedOffHours, unsigned long currentOffMs);

/// Mechanical DEC angle in degrees, from the DEC stepper position. 0 is the pole (home).
/// Negative values mean the DEC axis is turned through the pole: this is how the flipped slew solutions
/// (2 and 3 in calculateStepperPositions) leave the mount, and why currentRA() adds 12h for them.
float mechanicalDecDegrees(long decSteps, float stepsPerDECDegree, float zeroPosDEC);

/// True when the DEC axis is turned through the pole (mechanical DEC below 0).
bool isThroughThePole(float mechanicalDecDegrees);

/// Pointing state as ASCOM reports it in SideOfPier. ASCOM's normal state (pierEast) is the OTA on the east
/// side of the pier, counterweights down, observing west.
enum class PierSide
{
    East,
    West,
};

/// How the mount's home position relates to the sky, which decides what the firmware's direct slew solution is.
enum class MountGeometry
{
    /// OAT: home is not shifted, so the direct solution covers both sides of the meridian (like a fork mount).
    /// It is ASCOM's normal state; only the flipped solutions near the RA ring limits are through the pole.
    Tracker,
    /// OAM, OAE: home RA is shifted by 6h, so the mount works as a German equatorial: the direct solution is
    /// used east of the meridian (OTA west of the pier, observing east: pierWest) and the flipped one west of it
    /// (pierEast). With the default RA limits the switch happens about 1h before the meridian in the northern
    /// hemisphere (and about 1h after the meridian in the southern hemisphere).
    GermanEquatorial,
};

/// Side of pier from the mechanical DEC angle and the mount geometry. It changes only when the DEC axis is turned
/// through the pole, never while a target is merely tracked across the meridian.
PierSide pierSideFromMechanicalDec(float mechanicalDecDegrees, MountGeometry geometry);

/// Same, for a caller that already knows whether the DEC axis is through the pole.
PierSide pierSideFromPointingState(bool throughThePole, MountGeometry geometry);

}  // namespace core
