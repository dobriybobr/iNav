/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Unit tests for the TARGET_MODE attitude law (fc/target_guidance.c).
 *
 * The law is the only thing between a stream of correction quaternions and the plane's
 * control surfaces, and most of the ways it can be wrong look like "works, but oscillates"
 * rather than like a failure — so the cases below are the ones that cannot be diagnosed
 * from a flight log afterwards: axis signs, the >180 deg short arc, and above all the
 * closed loop (a q_sp recomposed every cycle instead of latched makes q_err identically
 * q_corr, which from the outside is indistinguishable from a badly tuned gain).
 */

#include <cmath>
#include <cstdint>

#include "gtest/gtest.h"

extern "C" {
#include "common/maths.h"
#include "common/quaternion.h"
#include "fc/target_guidance.h"
}

namespace {

constexpr float kAttKp = 8.0f;        // the app's IC_TGT_FC_KP
constexpr float kRateMax = 80.0f;     // IC_TGT_RATE_MAX
constexpr uint8_t kAllAxes = TG_AXIS_MASK_ROLL | TG_AXIS_MASK_PITCH | TG_AXIS_MASK_YAW;

targetGuidanceConfig_t defaultConfig()
{
    targetGuidanceConfig_t cfg = {};
    cfg.att_kp = kAttKp;
    cfg.rate_max_dps = kRateMax;
    cfg.lpf_hz = 0.0f;                // filtering off unless a test asks for it
    cfg.stale_ms = 350;
    cfg.hold_ms = 400;
    cfg.bank_max_deg = 45.0f;
    cfg.axis_mask = kAllAxes;
    return cfg;
}

fpQuaternion_t quatFrom(float rollDeg, float pitchDeg, float yawDeg)
{
    fpQuaternion_t q;
    targetGuidanceQuatFromEuler(rollDeg, pitchDeg, yawDeg, &q);
    return q;
}

// By-reference wrappers, so the cases below can be written with inline quatFrom() calls.
void errQuat(const fpQuaternion_t &orientation, const fpQuaternion_t &qSp, fpQuaternion_t *qErr)
{
    targetGuidanceErrorQuat(&orientation, &qSp, qErr);
}

// The rotation the law is driving to zero, as a single angle in degrees.
float errorAngleDeg(const fpQuaternion_t &orientation, const fpQuaternion_t &qSp)
{
    fpQuaternion_t qErr;
    errQuat(orientation, qSp, &qErr);
    return RADIANS_TO_DEGREES(2.0f * acosf(constrainf(fabsf(qErr.q0), -1.0f, 1.0f)));
}

// Signed difference between two headings, so 359.99997 and 0 are the same angle.
float headingDiffDeg(float a, float b)
{
    return fmodf(a - b + 540.0f, 360.0f) - 180.0f;
}

void configure(targetGuidance_t *tg, float lpfHz)
{
    targetGuidanceConfig_t cfg = defaultConfig();
    cfg.lpf_hz = lpfHz;
    targetGuidanceConfigure(tg, &cfg);
}

// Engaged module with one correction already latched.
void engage(targetGuidance_t *tg, const fpQuaternion_t &orientation, const fpQuaternion_t &qCorr,
            float gain = 1.0f, timeMs_t nowMs = 1000)
{
    targetGuidanceInit(tg);
    const targetGuidanceConfig_t cfg = defaultConfig();
    targetGuidanceConfigure(tg, &cfg);
    targetGuidanceSetAuthority(tg, true, true, false);
    targetGuidanceReceive(tg, nowMs, &orientation, &qCorr, gain);
}

} // namespace

// --- conventions ---------------------------------------------------------------------

// The euler <-> quaternion pair must be the exact mirror of imu.c, or every angle the law
// reads is off by an unknown rotation. Round-trip is the cheapest way to pin it.
TEST(TargetGuidanceTest, EulerRoundTrip)
{
    const float cases[][3] = {
        {0, 0, 0}, {10, 0, 0}, {0, 10, 0}, {0, 0, 10},
        {20, 10, 40}, {-35, 15, 350}, {44, -20, 180},
    };

    for (const auto &c : cases) {
        const fpQuaternion_t q = quatFrom(c[0], c[1], c[2]);
        float roll, pitch, yaw;
        targetGuidanceQuatToEuler(&q, &roll, &pitch, &yaw);

        EXPECT_NEAR(c[0], roll, 0.05f);
        EXPECT_NEAR(c[1], pitch, 0.05f);
        EXPECT_NEAR(0.0f, headingDiffDeg(c[2], yaw), 0.05f);
    }
}

// Roll and pitch come out with the sign the rate controller wants; yaw is MIRRORED because
// INAV builds the attitude quaternion with a negated yaw half-angle. Getting this wrong
// flies the plane away from the target on one axis only, which reads as a tracker bug.
TEST(TargetGuidanceTest, AxisSignsFollowInavConvention)
{
    const fpQuaternion_t level = quatFrom(0, 0, 0);
    float rate[3];

    fpQuaternion_t qErr;
    errQuat(level, quatFrom(10, 0, 0), &qErr);
    targetGuidanceErrorToRate(&qErr, kAttKp, 1.0f, 1000.0f, kAllAxes, rate);
    EXPECT_NEAR(kAttKp * 10.0f, rate[FD_ROLL], 0.5f);
    EXPECT_NEAR(0.0f, rate[FD_PITCH], 0.1f);
    EXPECT_NEAR(0.0f, rate[FD_YAW], 0.1f);

    errQuat(level, quatFrom(0, 10, 0), &qErr);
    targetGuidanceErrorToRate(&qErr, kAttKp, 1.0f, 1000.0f, kAllAxes, rate);
    EXPECT_NEAR(kAttKp * 10.0f, rate[FD_PITCH], 0.5f);
    EXPECT_NEAR(0.0f, rate[FD_ROLL], 0.1f);

    // +10 deg of heading must command a POSITIVE yaw rate even though q_err.q3 is negative.
    errQuat(level, quatFrom(0, 0, 10), &qErr);
    EXPECT_LT(qErr.q3, 0.0f);
    targetGuidanceErrorToRate(&qErr, kAttKp, 1.0f, 1000.0f, kAllAxes, rate);
    EXPECT_NEAR(kAttKp * 10.0f, rate[FD_YAW], 0.5f);
}

// --- the law -------------------------------------------------------------------------

TEST(TargetGuidanceTest, RateIsTheFormTheAppAssumes)
{
    // rate = att_kp * gain * err_deg, which is what the app's own rate cap is derived from.
    fpQuaternion_t qErr;
    errQuat(quatFrom(0, 0, 0), quatFrom(0, 5, 0), &qErr);

    float rate[3];
    targetGuidanceErrorToRate(&qErr, kAttKp, 0.5f, 1000.0f, kAllAxes, rate);
    EXPECT_NEAR(kAttKp * 0.5f * 5.0f, rate[FD_PITCH], 0.3f);

    targetGuidanceErrorToRate(&qErr, kAttKp, 0.0f, 1000.0f, kAllAxes, rate);
    EXPECT_FLOAT_EQ(0.0f, rate[FD_PITCH]);
}

TEST(TargetGuidanceTest, RateMaxClamps)
{
    fpQuaternion_t qErr;
    errQuat(quatFrom(0, 0, 0), quatFrom(0, 60, 0), &qErr);  // 8 * 60 = 480 dps raw

    float rate[3];
    targetGuidanceErrorToRate(&qErr, kAttKp, 1.0f, kRateMax, kAllAxes, rate);
    EXPECT_NEAR(kRateMax, rate[FD_PITCH], 0.001f);
}

TEST(TargetGuidanceTest, AxisMaskSilencesYaw)
{
    // The shipping configuration: yaw written but not used (plan R5).
    fpQuaternion_t qErr;
    errQuat(quatFrom(0, 0, 0), quatFrom(5, 5, 20), &qErr);

    float rate[3];
    targetGuidanceErrorToRate(&qErr, kAttKp, 1.0f, 1000.0f,
                              TG_AXIS_MASK_ROLL | TG_AXIS_MASK_PITCH, rate);
    EXPECT_FLOAT_EQ(0.0f, rate[FD_YAW]);
    EXPECT_GT(fabsf(rate[FD_ROLL]), 1.0f);
    EXPECT_GT(fabsf(rate[FD_PITCH]), 1.0f);
}

// 2*q_err.vec points the LONG way round when q0 < 0. Without the hemisphere flip the
// aircraft turns away from a setpoint more than 180 deg off and arrives eventually, so
// nothing in a log says "wrong".
TEST(TargetGuidanceTest, ShortArcBeyond180Degrees)
{
    const fpQuaternion_t orientation = quatFrom(0, 0, 0);
    const fpQuaternion_t qSp = quatFrom(0, 0, 190);  // == 170 deg to the LEFT

    fpQuaternion_t qErr;
    targetGuidanceErrorQuat(&orientation, &qSp, &qErr);
    EXPECT_GE(qErr.q0, 0.0f);
    EXPECT_NEAR(170.0f, errorAngleDeg(orientation, qSp), 0.5f);

    float rate[3];
    targetGuidanceErrorToRate(&qErr, kAttKp, 1.0f, kRateMax, kAllAxes, rate);
    EXPECT_LT(rate[FD_YAW], 0.0f) << "a 170 deg left turn must not be commanded to the right";
}

// --- latching: the one failure that looks like bad tuning ------------------------------

// With q_corr frozen, a setpoint LATCHED at receipt must converge as the aircraft rotates
// onto it. If q_sp were recomposed from the live attitude each cycle, q_err would stay
// equal to q_corr forever — an open loop (plan risk 8).
TEST(TargetGuidanceTest, LatchedSetpointConvergesAsAircraftRotates)
{
    const fpQuaternion_t start = quatFrom(0, 0, 0);
    const fpQuaternion_t qCorr = quatFrom(0, 20, 0);   // pitch up 20 deg, in body axes

    targetGuidance_t tg;
    engage(&tg, start, qCorr);

    float firstRate[3];
    ASSERT_EQ(TG_STAGE_FRESH, targetGuidanceUpdate(&tg, 1000, &start, 0.0f, 0.002f, firstRate));
    const float firstError = errorAngleDeg(start, tg.qSp);
    EXPECT_NEAR(20.0f, firstError, 0.5f);

    // Fly half way there; nothing new is received.
    const fpQuaternion_t halfway = quatFrom(0, 10, 0);
    float halfwayRate[3];
    ASSERT_EQ(TG_STAGE_FRESH, targetGuidanceUpdate(&tg, 1100, &halfway, 0.0f, 0.002f, halfwayRate));
    EXPECT_NEAR(10.0f, errorAngleDeg(halfway, tg.qSp), 0.5f);
    EXPECT_LT(fabsf(halfwayRate[FD_PITCH]), fabsf(firstRate[FD_PITCH]));

    // Arrive: the commanded rate must go to zero, not sit at its initial value.
    const fpQuaternion_t arrived = quatFrom(0, 20, 0);
    float arrivedRate[3];
    ASSERT_EQ(TG_STAGE_FRESH, targetGuidanceUpdate(&tg, 1200, &arrived, 0.0f, 0.002f, arrivedRate));
    EXPECT_NEAR(0.0f, errorAngleDeg(arrived, tg.qSp), 0.5f);
    EXPECT_LT(fabsf(arrivedRate[FD_PITCH]), 1.0f);
}

TEST(TargetGuidanceTest, SetpointIsComposedAgainstAttitudeAtReceipt)
{
    // Same correction received at two different attitudes must latch two different absolute
    // setpoints — that is what makes a correction behave like an absolute setpoint.
    const fpQuaternion_t qCorr = quatFrom(0, 10, 0);

    targetGuidance_t level, banked;
    engage(&level, quatFrom(0, 0, 0), qCorr);
    engage(&banked, quatFrom(30, 0, 0), qCorr);

    float levelRoll, bankedRoll;
    targetGuidanceQuatToEuler(&level.qSp, &levelRoll, nullptr, nullptr);
    targetGuidanceQuatToEuler(&banked.qSp, &bankedRoll, nullptr, nullptr);
    EXPECT_NEAR(0.0f, levelRoll, 0.5f);
    EXPECT_NEAR(30.0f, bankedRoll, 0.5f);
}

// --- authority gate --------------------------------------------------------------------

TEST(TargetGuidanceTest, AuthorityGate)
{
    const fpQuaternion_t level = quatFrom(0, 0, 0);
    targetGuidance_t tg;
    engage(&tg, level, quatFrom(0, 20, 0));
    float rate[3];

    EXPECT_TRUE(targetGuidanceIsActive(&tg));

    // Navigation or failsafe asking for the attitude wins: RTH and autoland fly through the
    // same angle path we bypass, so not releasing here is a plane that does not come home.
    targetGuidanceSetAuthority(&tg, true, true, true);
    EXPECT_FALSE(targetGuidanceIsActive(&tg));
    EXPECT_EQ(TG_STAGE_IDLE, targetGuidanceUpdate(&tg, 1000, &level, 0.0f, 0.002f, rate));
    EXPECT_FLOAT_EQ(0.0f, rate[FD_PITCH]);

    // Box down and disarmed are the other two ways out, and the box must work even if
    // everything else is stuck (plan risk 10).
    targetGuidanceSetAuthority(&tg, true, false, false);
    EXPECT_FALSE(targetGuidanceIsActive(&tg));
    targetGuidanceSetAuthority(&tg, false, true, false);
    EXPECT_FALSE(targetGuidanceIsActive(&tg));

    targetGuidanceSetAuthority(&tg, true, true, false);
    EXPECT_EQ(TG_STAGE_FRESH, targetGuidanceUpdate(&tg, 1000, &level, 0.0f, 0.002f, rate));
}

TEST(TargetGuidanceTest, NothingLatchedIsNotActive)
{
    targetGuidance_t tg;
    targetGuidanceInit(&tg);
    const targetGuidanceConfig_t cfg = defaultConfig();
    targetGuidanceConfigure(&tg, &cfg);
    targetGuidanceSetAuthority(&tg, true, true, false);

    EXPECT_FALSE(targetGuidanceIsActive(&tg));
}

// --- the three staleness stages ---------------------------------------------------------

TEST(TargetGuidanceTest, StageForAge)
{
    EXPECT_EQ(TG_STAGE_FRESH, targetGuidanceStageForAge(0, 350, 400));
    EXPECT_EQ(TG_STAGE_FRESH, targetGuidanceStageForAge(350, 350, 400));
    EXPECT_EQ(TG_STAGE_HOLD, targetGuidanceStageForAge(351, 350, 400));
    EXPECT_EQ(TG_STAGE_HOLD, targetGuidanceStageForAge(750, 350, 400));
    EXPECT_EQ(TG_STAGE_LEVEL_HOLD, targetGuidanceStageForAge(751, 350, 400));
    // hold_ms = 0 means "go straight to level-hold"
    EXPECT_EQ(TG_STAGE_LEVEL_HOLD, targetGuidanceStageForAge(351, 350, 0));
}

// The axes are never handed back to the pilot: a centred stick in ACRO is zero rate, not
// level, so the bank would simply stay (plan R3).
TEST(TargetGuidanceTest, StaleStreamConvergesToLevelAndStaysActive)
{
    const fpQuaternion_t banked = quatFrom(35, 0, 0);
    targetGuidance_t tg;
    engage(&tg, banked, quatFrom(0, 0, 0), 1.0f, 1000);  // latch the bank it is already in

    float rate[3];
    EXPECT_EQ(TG_STAGE_FRESH, targetGuidanceUpdate(&tg, 1300, &banked, 0.0f, 0.002f, rate));
    EXPECT_LT(fabsf(rate[FD_ROLL]), 1.0f) << "on the latched setpoint there is nothing to do";

    // Stream lost: still active, now rolling back out of the bank.
    EXPECT_EQ(TG_STAGE_HOLD, targetGuidanceUpdate(&tg, 1600, &banked, 0.0f, 0.002f, rate));
    EXPECT_TRUE(targetGuidanceIsActive(&tg));
    EXPECT_LT(rate[FD_ROLL], 0.0f) << "a +35 deg bank must be commanded back toward level";

    // Past hold_ms the setpoint IS level, and the command is proportional to the full bank.
    EXPECT_EQ(TG_STAGE_LEVEL_HOLD, targetGuidanceUpdate(&tg, 2000, &banked, 0.0f, 0.002f, rate));
    EXPECT_NEAR(-kRateMax, rate[FD_ROLL], 0.001f);  // 8 * 35 = 280 dps, clamped

    float roll;
    targetGuidanceQuatToEuler(&banked, &roll, nullptr, nullptr);
    EXPECT_NEAR(35.0f, roll, 0.5f);  // and the aircraft itself has not moved in this test
}

TEST(TargetGuidanceTest, HoldBlendIsMonotonic)
{
    const fpQuaternion_t banked = quatFrom(40, 0, 0);
    targetGuidance_t tg;
    engage(&tg, banked, quatFrom(0, 0, 0), 1.0f, 1000);

    // Through the blend the commanded setpoint's bank must only ever decrease.
    float previous = 999.0f;
    for (timeMs_t t = 1350; t <= 1750; t += 50) {
        float rate[3];
        targetGuidanceUpdate(&tg, t, &banked, 0.0f, 0.002f, rate);
        const float commandedBank = 40.0f - fabsf(rate[FD_ROLL]) / kAttKp;  // err = bank still to remove
        EXPECT_LE(commandedBank, previous + 0.01f);
        previous = commandedBank;
    }
}

// --- level trim --------------------------------------------------------------------------

// INAV applies the trim as `angleTarget -= DEGREES_TO_DECIDEGREES(fixedWingLevelTrim)` and
// spends ten lines of comment on how counter-intuitive that is (positive trim = nose up,
// positive rcCommand = nose down). Level-hold must reproduce the EXPRESSION, not the
// meaning, or the plane holds the wrong pitch by twice the trim.
TEST(TargetGuidanceTest, LevelQuatUsesInavTrimSign)
{
    const fpQuaternion_t orientation = quatFrom(10, 5, 123);
    fpQuaternion_t qLevel;
    targetGuidanceLevelQuat(&orientation, 3.0f, &qLevel);

    float roll, pitch, yaw;
    targetGuidanceQuatToEuler(&qLevel, &roll, &pitch, &yaw);

    EXPECT_NEAR(0.0f, roll, 0.05f);
    EXPECT_NEAR(-3.0f, pitch, 0.05f);
    EXPECT_NEAR(0.0f, headingDiffDeg(123.0f, yaw), 0.1f) << "level-hold must not turn the aircraft";
}

// --- bank limit ----------------------------------------------------------------------------

// The clamp is on the SETPOINT, not on the output rate: clamping the rate would leave the
// limiter and the law fighting each other, and after bypassing pidLevel there is no
// max_angle_inclination behind us any more.
TEST(TargetGuidanceTest, BankClampLimitsTheSetpointAndKeepsPitchAndHeading)
{
    fpQuaternion_t qSp = quatFrom(70, 8, 200);
    targetGuidanceClampBank(&qSp, 45.0f);

    float roll, pitch, yaw;
    targetGuidanceQuatToEuler(&qSp, &roll, &pitch, &yaw);
    EXPECT_NEAR(45.0f, roll, 0.2f);
    EXPECT_NEAR(8.0f, pitch, 0.2f);
    EXPECT_NEAR(0.0f, headingDiffDeg(200.0f, yaw), 0.2f);

    // ... and in the other direction, and not at all when already inside the limit.
    qSp = quatFrom(-70, 0, 0);
    targetGuidanceClampBank(&qSp, 45.0f);
    targetGuidanceQuatToEuler(&qSp, &roll, nullptr, nullptr);
    EXPECT_NEAR(-45.0f, roll, 0.2f);

    qSp = quatFrom(20, 0, 0);
    targetGuidanceClampBank(&qSp, 45.0f);
    targetGuidanceQuatToEuler(&qSp, &roll, nullptr, nullptr);
    EXPECT_NEAR(20.0f, roll, 0.2f);
}

TEST(TargetGuidanceTest, BankClampAppliesAtReceipt)
{
    // Even with the app dead or lying, the limit is a property of the FC.
    targetGuidance_t tg;
    engage(&tg, quatFrom(0, 0, 0), quatFrom(80, 0, 0));

    float roll;
    targetGuidanceQuatToEuler(&tg.qSp, &roll, nullptr, nullptr);
    EXPECT_NEAR(45.0f, roll, 0.2f);
}

// --- PT1 on the commanded rate (R7) ------------------------------------------------------

// The filter pidLevel used to apply and that bypassing pidLevel removes. It must damp the
// onset without changing the steady-state value, because the app's rate cap is derived from
// rate = kp * gain * err.
TEST(TargetGuidanceTest, RateFilterDampsOnsetButKeepsSteadyState)
{
    const fpQuaternion_t level = quatFrom(0, 0, 0);
    const fpQuaternion_t qCorr = quatFrom(0, 5, 0);

    targetGuidance_t tg;
    engage(&tg, level, qCorr);
    configure(&tg, 7.0f);

    const float dT = 0.002f;
    float rate[3];
    targetGuidanceUpdate(&tg, 1000, &level, 0.0f, dT, rate);
    const float firstCycle = rate[FD_PITCH];
    EXPECT_LT(firstCycle, kAttKp * 5.0f * 0.5f) << "the first cycle must not be the full step";

    for (int i = 0; i < 2000; i++) {   // 4 s of PID cycles, no new correction, attitude frozen
        targetGuidanceUpdate(&tg, 1000, &level, 0.0f, dT, rate);
    }
    EXPECT_NEAR(kAttKp * 5.0f, rate[FD_PITCH], 0.5f) << "steady state must be kp * err";
}

// A filter still holding the tail of the previous engagement would put a step into the
// first cycle of the next one.
TEST(TargetGuidanceTest, RateFilterResetsOnEngage)
{
    const fpQuaternion_t level = quatFrom(0, 0, 0);
    targetGuidance_t tg;
    engage(&tg, level, quatFrom(0, 30, 0));
    configure(&tg, 7.0f);

    float rate[3];
    for (int i = 0; i < 500; i++) {
        targetGuidanceUpdate(&tg, 1000, &level, 0.0f, 0.002f, rate);
    }
    ASSERT_GT(rate[FD_PITCH], 10.0f);

    targetGuidanceSetAuthority(&tg, true, false, false);   // box down
    targetGuidanceUpdate(&tg, 1000, &level, 0.0f, 0.002f, rate);
    targetGuidanceSetAuthority(&tg, true, true, false);    // box up again

    const fpQuaternion_t smallCorr = quatFrom(0, 1, 0);
    targetGuidanceReceive(&tg, 2000, &level, &smallCorr, 1.0f);
    targetGuidanceUpdate(&tg, 2000, &level, 0.0f, 0.002f, rate);
    EXPECT_LT(rate[FD_PITCH], kAttKp * 1.0f) << "re-engage must start from zero, not from the old tail";
}

// --- debug[7] path marker -----------------------------------------------------------------

// The app classifies this marker by RANGE and stops steering on 1500..2500 ("the FC fell
// back to its manual stick law"). This firmware has no such fallback, so holding and
// level-hold must stay out of that band — claiming a fault that cannot happen here would
// withhold guidance for the rest of the flight.
TEST(TargetGuidanceTest, PathMarkersAvoidTheAppsManualLawBand)
{
    const fpQuaternion_t level = quatFrom(0, 0, 0);
    float rate[3];
    targetGuidance_t tg;

    // engaged but nothing ever received
    targetGuidanceInit(&tg);
    const targetGuidanceConfig_t cfg = defaultConfig();
    targetGuidanceConfigure(&tg, &cfg);
    targetGuidanceSetAuthority(&tg, true, true, false);
    targetGuidanceUpdate(&tg, 1000, &level, 0.0f, 0.002f, rate);
    EXPECT_EQ(TG_PATH_NO_SETPOINT, targetGuidancePathMarker(&tg));

    engage(&tg, level, quatFrom(0, 10, 0), 1.0f, 1000);
    targetGuidanceUpdate(&tg, 1000, &level, 0.0f, 0.002f, rate);
    EXPECT_EQ(TG_PATH_FRESH, targetGuidancePathMarker(&tg));

    targetGuidanceUpdate(&tg, 1500, &level, 0.0f, 0.002f, rate);
    EXPECT_EQ(TG_PATH_HOLD, targetGuidancePathMarker(&tg));

    targetGuidanceUpdate(&tg, 2500, &level, 0.0f, 0.002f, rate);
    EXPECT_EQ(TG_PATH_LEVEL_HOLD, targetGuidancePathMarker(&tg));

    targetGuidanceSetAuthority(&tg, true, true, true);
    targetGuidanceUpdate(&tg, 2500, &level, 0.0f, 0.002f, rate);
    EXPECT_EQ(TG_PATH_GATED, targetGuidancePathMarker(&tg));

    targetGuidanceSetAuthority(&tg, true, false, false);
    targetGuidanceUpdate(&tg, 2500, &level, 0.0f, 0.002f, rate);
    EXPECT_EQ(TG_PATH_IDLE, targetGuidancePathMarker(&tg));

    for (int marker : {TG_PATH_HOLD, TG_PATH_LEVEL_HOLD, TG_PATH_GATED}) {
        EXPECT_FALSE(marker >= 1500 && marker < 2500) << marker << " reads as the manual law";
        EXPECT_FALSE(marker > 900 && marker < 1100) << marker << " reads as a fresh setpoint";
    }
}
