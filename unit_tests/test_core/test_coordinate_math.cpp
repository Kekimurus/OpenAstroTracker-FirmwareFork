#include <gtest/gtest.h>
#include "core/CoordinateMath.hpp"

using core::calculateStepperPositions;
using core::SlewGeometry;
using core::SlewResult;

// Helper: create a default geometry for testing
static SlewGeometry makeGeo(float moveRA, float moveDEC, float deltaRA)
{
    SlewGeometry geo;
    geo.moveRA               = moveRA;
    geo.moveDEC              = moveDEC;
    geo.homeTargetDeltaRA    = deltaRA;
    geo.stepsPerSiderealHour = 1000.0f;
    geo.stepsPerDECDegree    = 500.0f;
    geo.zeroPosDEC           = 0.0f;
    geo.raLimitL             = -6.0f;
    geo.raLimitR             = 6.0f;
    return geo;
}

TEST(CoordinateMathTest, Solution1Direct)
{
    // Target within limits: no flip needed
    SlewGeometry geo  = makeGeo(3.0f, 45.0f, 2.0f);
    SlewResult result = calculateStepperPositions(geo);

    // moveRA = 3.0, moveDEC = 45.0
    EXPECT_EQ(static_cast<long>(-3.0f * 1000.0f), result.targetRASteps);
    EXPECT_EQ(static_cast<long>(45.0f * 500.0f), result.targetDECSteps);
}

TEST(CoordinateMathTest, Solution2FlipUpper)
{
    // Past upper limit: flip (moveRA -= 12, moveDEC = -moveDEC)
    SlewGeometry geo  = makeGeo(4.0f, 30.0f, 7.0f);  // deltaRA=7 > raLimitR=6
    SlewResult result = calculateStepperPositions(geo);

    // moveRA = 4 - 12 = -8, moveDEC = -30
    EXPECT_EQ(static_cast<long>(-(-8.0f) * 1000.0f), result.targetRASteps);
    EXPECT_EQ(static_cast<long>((-30.0f) * 500.0f), result.targetDECSteps);
}

TEST(CoordinateMathTest, Solution3FlipLower)
{
    // Past lower limit: flip (moveRA += 12, moveDEC = -moveDEC)
    SlewGeometry geo  = makeGeo(-4.0f, 20.0f, -7.0f);  // deltaRA=-7 < raLimitL=-6
    SlewResult result = calculateStepperPositions(geo);

    // moveRA = -4 + 12 = 8, moveDEC = -20
    EXPECT_EQ(static_cast<long>(-8.0f * 1000.0f), result.targetRASteps);
    EXPECT_EQ(static_cast<long>((-20.0f) * 500.0f), result.targetDECSteps);
}

TEST(CoordinateMathTest, ZeroPosDecOffset)
{
    SlewGeometry geo  = makeGeo(2.0f, 30.0f, 1.0f);
    geo.zeroPosDEC    = 5.0f;  // accumulated sync offset
    SlewResult result = calculateStepperPositions(geo);

    // moveDEC = 30 - 5 = 25
    EXPECT_EQ(static_cast<long>(25.0f * 500.0f), result.targetDECSteps);
}

TEST(CoordinateMathTest, SolutionsArray)
{
    SlewGeometry geo  = makeGeo(2.0f, 45.0f, 1.0f);
    SlewResult result = calculateStepperPositions(geo);

    // Solution 1
    EXPECT_EQ(static_cast<long>(-2.0f * 1000.0f), result.solutions[0]);
    EXPECT_EQ(static_cast<long>(45.0f * 500.0f), result.solutions[1]);
    // Solution 2
    EXPECT_EQ(static_cast<long>(-(2.0f - 12.0f) * 1000.0f), result.solutions[2]);
    EXPECT_EQ(static_cast<long>(-45.0f * 500.0f), result.solutions[3]);
    // Solution 3
    EXPECT_EQ(static_cast<long>(-(2.0f + 12.0f) * 1000.0f), result.solutions[4]);
    EXPECT_EQ(static_cast<long>(-45.0f * 500.0f), result.solutions[5]);
}

TEST(CoordinateMathTest, BoundaryExactlyAtLimit)
{
    // Exactly at the limit: no flip (only > triggers, not >=)
    SlewGeometry geo  = makeGeo(3.0f, 30.0f, 6.0f);
    SlewResult result = calculateStepperPositions(geo);

    // Should use solution 1 (direct)
    EXPECT_EQ(static_cast<long>(-3.0f * 1000.0f), result.targetRASteps);
    EXPECT_EQ(static_cast<long>(30.0f * 500.0f), result.targetDECSteps);
}

// Expected values are derived from the length of the sidereal day (86164.0905 s), not from the code.
TEST(UntrackedRaHoursTest, ZeroWhileTrackingWithNoOffTime)
{
    EXPECT_FLOAT_EQ(0.0f, core::untrackedRaHours(0.0f, 0UL));
}

TEST(UntrackedRaHoursTest, RunningOffPeriodAdvancesAtSiderealRate)
{
    // 30 s stopped (FW12, night 2): RA must advance 30 s * 86400 / 86164.0905 = 30.0821 s
    EXPECT_NEAR(30.0821f / 3600.0f, core::untrackedRaHours(0.0f, 30000UL), 1e-6f);
    // One sidereal day stopped is exactly 24 h of RA
    EXPECT_NEAR(24.0f, core::untrackedRaHours(0.0f, 86164091UL), 1e-4f);
}

TEST(UntrackedRaHoursTest, CompletedAndRunningOffTimeAdd)
{
    // 1 h completed + 1 h running = 2 solar hours = 2.0054758 RA hours
    EXPECT_NEAR(2.0054758f, core::untrackedRaHours(1.0f, 3600000UL), 1e-5f);
}

TEST(UntrackedRaHoursTest, CatchUpCancelsTheSlewTime)
{
    // A 20 s slew whose catch-up (20 s) has already been subtracted leaves nothing
    EXPECT_NEAR(0.0f, core::untrackedRaHours(-20.0f / 3600.0f, 20000UL), 1e-6f);
}

// ---- Side of pier (:Gm#) ----------------------------------------------------

using core::isThroughThePole;
using core::mechanicalDecDegrees;
using core::MountGeometry;
using core::PierSide;
using core::pierSideFromMechanicalDec;
using core::pierSideFromPointingState;

namespace
{
const float kDecStepsPerDegree = 500.0f;

// Side of pier after a slew, as the firmware would report it: the solution selector decides the DEC steps,
// the DEC sign decides the pointing state, the geometry turns that into an ASCOM side.
PierSide sideAfterSlew(const SlewGeometry &geo, MountGeometry geometry)
{
    SlewResult result = calculateStepperPositions(geo);
    return pierSideFromMechanicalDec(mechanicalDecDegrees(result.targetDECSteps, kDecStepsPerDegree, 0.0f), geometry);
}

// Geometry for a target at hour angle `ha` with the firmware's default northern limits (left 5h, right 7h).
// OAM/OAE shift their home RA by 6h (Mount::setHome), so the delta to home is -ha - 6 there; on the OAT it is -ha.
SlewGeometry atHourAngle(float ha, float homeShiftHours)
{
    float delta = -ha - homeShiftHours;
    while (delta > 12.0f)
        delta -= 24.0f;
    while (delta < -12.0f)
        delta += 24.0f;
    SlewGeometry geo = makeGeo(delta, 45.0f, delta);
    geo.raLimitL     = -5.0f;
    geo.raLimitR     = 7.0f;
    return geo;
}
}  // namespace

TEST(PierSideTest, MechanicalDecFromStepsAndSyncOffset)
{
    EXPECT_FLOAT_EQ(45.0f, mechanicalDecDegrees(22500, 500.0f, 0.0f));
    EXPECT_FLOAT_EQ(-30.0f, mechanicalDecDegrees(-15000, 500.0f, 0.0f));
    // A sync offset moves the zero, as in currentRA()/currentDEC().
    EXPECT_FLOAT_EQ(-1.0f, mechanicalDecDegrees(500, 500.0f, -2.0f));
}

TEST(PierSideTest, ThroughThePoleIsNegativeMechanicalDec)
{
    EXPECT_FALSE(isThroughThePole(0.0f));  // home
    EXPECT_FALSE(isThroughThePole(60.0f));
    EXPECT_TRUE(isThroughThePole(-0.01f));
}

TEST(PierSideTest, GeometryDecidesWhichStateIsWhichSide)
{
    EXPECT_EQ(PierSide::East, pierSideFromPointingState(false, MountGeometry::Tracker));
    EXPECT_EQ(PierSide::West, pierSideFromPointingState(true, MountGeometry::Tracker));
    EXPECT_EQ(PierSide::West, pierSideFromPointingState(false, MountGeometry::GermanEquatorial));
    EXPECT_EQ(PierSide::East, pierSideFromPointingState(true, MountGeometry::GermanEquatorial));
    EXPECT_EQ(pierSideFromPointingState(true, MountGeometry::GermanEquatorial),
              pierSideFromMechanicalDec(-10.0f, MountGeometry::GermanEquatorial));
}

TEST(PierSideTest, OatDirectSolutionIsNormalOnBothSidesOfTheMeridian)
{
    for (float ha : {-4.0f, -0.5f, 0.5f, 4.0f})
    {
        EXPECT_EQ(PierSide::East, sideAfterSlew(atHourAngle(ha, 0.0f), MountGeometry::Tracker)) << "HA " << ha;
    }
    // Beyond the RA ring limits the firmware goes through the pole.
    EXPECT_EQ(PierSide::West, sideAfterSlew(atHourAngle(-8.0f, 0.0f), MountGeometry::Tracker));
    EXPECT_EQ(PierSide::West, sideAfterSlew(atHourAngle(5.5f, 0.0f), MountGeometry::Tracker));
}

TEST(PierSideTest, OamOaeSlewLikeAGermanEquatorial)
{
    // ASCOM: observing east = OTA on the west side (pierWest), observing west = pierEast.
    for (float ha : {-8.0f, -4.0f, -1.5f})
    {
        EXPECT_EQ(PierSide::West, sideAfterSlew(atHourAngle(ha, 6.0f), MountGeometry::GermanEquatorial)) << "HA " << ha;
    }
    for (float ha : {-0.5f, 0.5f, 3.0f, 5.5f})
    {
        EXPECT_EQ(PierSide::East, sideAfterSlew(atHourAngle(ha, 6.0f), MountGeometry::GermanEquatorial)) << "HA " << ha;
    }
}

TEST(PierSideTest, TrackingAwayFromTheSlewPointDoesNotChangeTheSide)
{
    // Tracking only turns the RA ring; the DEC axis keeps the steps of the solution it was slewed to. The side
    // reported must therefore stay put until the next slew, whatever the hour angle does meanwhile.
    SlewResult slewed = calculateStepperPositions(atHourAngle(-0.5f, 0.0f));  // OAT, just east of the meridian
    const float dec   = mechanicalDecDegrees(slewed.targetDECSteps, kDecStepsPerDegree, 0.0f);
    EXPECT_EQ(PierSide::East, pierSideFromMechanicalDec(dec, MountGeometry::Tracker));
    // A slew to the same target an hour later (now west of the meridian) still picks the direct solution.
    EXPECT_EQ(slewed.targetDECSteps, calculateStepperPositions(atHourAngle(0.5f, 0.0f)).targetDECSteps);
}
