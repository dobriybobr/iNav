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
 * target_guidance — the plane's TARGET_MODE attitude law (VOT_C B2).
 *
 * The aircraft is flown on a stream of BODY-FRAME correction quaternions (MSP 231,
 * SetTargetCorrection): the app sends q_corr + gain, we compose the absolute setpoint
 * ONCE at receipt against the fresh IMU attitude (`q_sp = orientation (x) q_corr`),
 * latch it, and then close the attitude loop ourselves every PID cycle:
 *
 *     q_err = conj(orientation) (x) q_sp_latched
 *     rate_dps[axis] = att_kp * gain * (2 * q_err.vec[axis] in degrees)
 *
 * Because q_sp is latched, q_err is NOT conj(o)(x)o(x)q_corr == q_corr (the open-loop
 * degeneracy); instead q_sp stays fixed in space and the error converges as the aircraft
 * rotates onto it. This is exactly the trick that makes a body-frame correction behave
 * like an absolute setpoint built on the FC's own fresh attitude.
 *
 * Deliberately decoupled from MSP/pid/settings so the math is unit-testable with an
 * injected clock and orientation; the pid.c hooks and the MSP 231 handler are thin
 * callers that live elsewhere.
 *
 * AXIS SIGNS ARE NOT SYMMETRIC. INAV builds its attitude quaternion with a NEGATED yaw
 * half-angle (imu.c::imuComputeQuaternionFromRPY), so a setpoint that is +10 deg to the
 * right in heading shows up as a NEGATIVE q_err.q3. Roll and pitch come out with the
 * sign the rate controller wants, yaw does not — see TG_AXIS_SIGN. Verified numerically
 * against the mirrored formula before this module was written, and pinned by
 * targetGuidanceErrorToRate's unit test; do not "clean it up".
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/filter.h"
#include "common/quaternion.h"
#include "common/time.h"

// Sign that maps 2*q_err.vec[axis] onto a body rate in INAV's convention, i.e. the rate
// whose positive direction increases attitude.raw[axis]. Yaw is mirrored, see the header
// comment above.
#define TG_AXIS_SIGN_ROLL   (+1.0f)
#define TG_AXIS_SIGN_PITCH  (+1.0f)
#define TG_AXIS_SIGN_YAW    (-1.0f)

#define TG_AXIS_MASK_ROLL   (1 << 0)
#define TG_AXIS_MASK_PITCH  (1 << 1)
#define TG_AXIS_MASK_YAW    (1 << 2)

/*
 * debug[7] path markers, published under debug_mode = TARGET.
 *
 * The values are NOT free to choose: the app already classifies this marker by range
 * (DroneMsp.cpp CheckTargetSetpointAccepted) — 900..1100 means "autonomous law", and
 * 1500..2500 means "the FC fell back to its manual stick law", which is the reading that
 * makes it stop steering. On iNav there is no manual fallback at all (the module holds and
 * then levels, plan R3), so HOLD and LEVEL_HOLD must not land in that band and claim a
 * fault that cannot happen here. They go in the gap at 1200/1300, where an app that has not
 * learned them yet reports "cannot confirm" instead of a wrong diagnosis.
 *
 * 2100 keeps its Betaflight meaning exactly: no setpoint has ever arrived.
 */
#define TG_PATH_IDLE        0       // module not engaged (disarmed or box down)
#define TG_PATH_FRESH       1000    // following a fresh latched setpoint
#define TG_PATH_HOLD        1200    // stale: latched setpoint converging to level
#define TG_PATH_LEVEL_HOLD  1300    // stale past hold_ms: the FC's own level-hold
#define TG_PATH_GATED       1400    // failsafe or navigation owns the attitude (plan R4)
#define TG_PATH_NO_SETPOINT 2100    // engaged, but nothing has ever been received

// debug slot layout, mirroring what the app already decodes (DroneMsp.cpp onDebug):
//   [0..2] commanded body rate, deg/s      [3] gain x1000
//   [4] attitude error, deg x10            [5] (now_ms / 8) & 0xffff, aligns with blackbox
//   [6] age of the latched setpoint, ms    [7] TG_PATH_*
#define TG_DEBUG_GAIN_SCALE  1000.0f
#define TG_DEBUG_ERR_SCALE   10.0f

typedef enum {
    TG_STAGE_IDLE = 0,      // no latched setpoint, or no authority (disarmed/gated)
    TG_STAGE_FRESH,         // following the latched setpoint
    TG_STAGE_HOLD,          // stale: blending the latched setpoint toward level
    TG_STAGE_LEVEL_HOLD,    // stale past hold_ms: wings level + level-trim pitch
} targetGuidanceStage_e;

typedef struct targetGuidanceConfig_s {
    float    att_kp;          // dps per degree of attitude error (8.0; the app's IC_TGT_FC_KP)
    float    rate_max_dps;    // clamp on commanded body rate (only ceiling after bypassing pidLevel)
    float    lpf_hz;          // PT1 cutoff on the commanded rate, 0 = off (replaces pidLevel's
                              // PID_LEVEL.I filter, which bypassing pidLevel would otherwise lose)
    uint16_t stale_ms;        // fresh -> hold
    uint16_t hold_ms;         // hold -> level-hold (blend duration after going stale)
    float    bank_max_deg;    // setpoint bank clamp, applied to q_sp (not to the output rate)
    uint8_t  axis_mask;       // TG_AXIS_MASK_*
} targetGuidanceConfig_t;

typedef struct targetGuidance_s {
    targetGuidanceConfig_t cfg;

    bool     latched;               // a setpoint has been received at least once
    fpQuaternion_t qSp;             // absolute setpoint, composed ONCE at receipt from q_corr
    float    gain;                  // [0..1], from the app
    timeMs_t lastUpdateMs;          // receipt time

    // Authority, refreshed by the caller every cycle — never latched as an event, so a
    // wedged module cannot stay active (plan risk 10).
    bool     armed;
    bool     boxTarget;
    bool     higherAuthority;       // failsafe or navigation owns the attitude

    // Runtime, owned by targetGuidanceUpdate()
    targetGuidanceStage_e stage;
    bool     wasActive;             // edge detect, resets the rate filters on 0 -> 1
    pt1Filter_t rateFilter[3];
} targetGuidance_t;

void targetGuidanceInit(targetGuidance_t *tg);
void targetGuidanceConfigure(targetGuidance_t *tg, const targetGuidanceConfig_t *cfg);

// MSP 231 handler: latch a new correction against the CURRENT orientation.
void targetGuidanceReceive(targetGuidance_t *tg, timeMs_t nowMs,
                           const fpQuaternion_t *orientation,
                           const fpQuaternion_t *qCorr, float gain);

// Authority from the caller, recomputed every PID cycle.
void targetGuidanceSetAuthority(targetGuidance_t *tg, bool armed, bool boxTarget, bool higherAuthority);

// True when the module owns the attitude THIS cycle (the pid.c hooks test this).
bool targetGuidanceIsActive(const targetGuidance_t *tg);

// Per-PID-cycle law. Fills rateDps[3] (roll/pitch/yaw, deg/s) and returns the stage.
// levelTrimPitchDeg is getFixedWingLevelTrim() in the real hook, a constant in tests.
// Returns TG_STAGE_IDLE with rateDps zeroed when the module has no authority.
targetGuidanceStage_e targetGuidanceUpdate(targetGuidance_t *tg, timeMs_t nowMs,
                                           const fpQuaternion_t *orientation,
                                           float levelTrimPitchDeg, float dT,
                                           float rateDps[3]);

// --- pure kernel (unit-tested, no FC state, no clock, no settings globals) ---

// q_err = conj(orientation) (x) qSp, normalized, shortest arc (q0 >= 0).
// The shortest-arc flip is MANDATORY: 2*q_err.vec only approximates the rotation vector
// for q0 > 0; at q0 < 0 it points the LONG way round (plan 4.1).
void targetGuidanceErrorQuat(const fpQuaternion_t *orientation, const fpQuaternion_t *qSp,
                             fpQuaternion_t *qErr);

// Body rates from the error quaternion. Uses 2*q_err.vec (exact to second order, which is
// where a plane lives) instead of quaternionToAxisAngle — that function is called nowhere
// in INAV and is not the inverse of axisAngleToQuaternion (it inverts the axis).
void targetGuidanceErrorToRate(const fpQuaternion_t *qErr, float attKp, float gain,
                               float rateMaxDps, uint8_t axisMask, float rateDps[3]);

// Euler angles of an attitude quaternion, degrees, the exact mirror of
// imu.c::imuUpdateEulerAngles (atan2 of the same rMat entries).
void targetGuidanceQuatToEuler(const fpQuaternion_t *q, float *rollDeg, float *pitchDeg, float *yawDeg);

// Attitude quaternion for (rollDeg, pitchDeg, yawDeg), ZYX with the negated yaw
// half-angle — the exact mirror of imu.c::imuComputeQuaternionFromRPY.
void targetGuidanceQuatFromEuler(float rollDeg, float pitchDeg, float yawDeg, fpQuaternion_t *q);

// Clamps the bank of a setpoint quaternion to +-bankMaxDeg, keeping its pitch and heading.
// Applied to the SETPOINT at receipt, not to the output rate, so the limiter and the law
// cannot fight each other (plan 4.1 p.5). bankMaxDeg <= 0 disables the clamp.
void targetGuidanceClampBank(fpQuaternion_t *qSp, float bankMaxDeg);

// The level-hold setpoint: wings level, pitch = -levelTrimPitchDeg, heading unchanged.
// The pitch sign reproduces INAV's own expression (pid.c computePidLevelTarget:
// `angleTarget -= DEGREES_TO_DECIDEGREES(fixedWingLevelTrim)` with angleTarget 0), which
// its ten-line comment calls counter-intuitive. Do not "fix" it by meaning.
void targetGuidanceLevelQuat(const fpQuaternion_t *orientation, float levelTrimPitchDeg,
                             fpQuaternion_t *qOut);

// Shortest-arc interpolation between two setpoints, t in [0..1] (normalized lerp — the
// stage blend is sub-degree per cycle, where nlerp and slerp are indistinguishable).
void targetGuidanceBlendQuat(const fpQuaternion_t *qFrom, const fpQuaternion_t *qTo, float t,
                             fpQuaternion_t *qOut);

// Stage from the age of the latched setpoint. Pure, so the three transitions are testable
// without a clock.
targetGuidanceStage_e targetGuidanceStageForAge(timeDelta_t ageMs, uint16_t staleMs, uint16_t holdMs);

// The debug[7] marker for the module's current state. Pure, and separate from the stage
// because "gated" and "nothing received yet" are not stages of the law — they are the two
// states in which the law does not run at all, and the app has to tell them apart from a
// marker that is simply not being published.
int16_t targetGuidancePathMarker(const targetGuidance_t *tg);
