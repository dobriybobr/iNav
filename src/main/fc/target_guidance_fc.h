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
 * The FC-side wrapper around the TARGET_MODE attitude law.
 *
 * Everything that reads a global -- orientation, the arming flags, the TARGET box, the
 * failsafe and navigation authority, the level trim, pidProfile -- lives HERE, and the law
 * itself (fc/target_guidance.c) takes all of it as parameters. That split is what keeps the
 * law unit-testable: its test links common/maths.c and nothing else, while this file would
 * drag in flight/imu.c and flight/pid.c, whose harnesses in this tree are disabled.
 *
 * The law runs ONCE per PID cycle, from targetGuidanceFcUpdate() in taskMainPidLoop, and
 * pid.c only reads the cached result. It has two hook sites (roll/pitch in step 3, yaw in
 * step 2), so computing it inside either would either run it twice or make the two axes see
 * different attitudes.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/time.h"

// MSP 233 handshake. protocol 3 says "this firmware closes the attitude loop with the
// q_err law"; the app hard-stops on anything lower rather than guessing, which is the
// lesson of 2026-09-07.
#define TG_MSP_PROTOCOL_VERSION     3
#define TG_MSP_SUPPORTS_CORRECTION  (1u << 1)   // MSP 231 accepted
#define TG_MSP_SUPPORTS_QERR_LAW    (1u << 3)   // ... and driven by the q_err law, not an
                                                // angle-mode stick emulation (bit2 is the
                                                // Betaflight fork's reserved 232)

// Called once per PID cycle from taskMainPidLoop, before pidController().
void targetGuidanceFcUpdate(timeUs_t currentTimeUs, float dT);

// MSP 231 handler. Quaternion components and gain already descaled to floats.
void targetGuidanceFcReceive(float qw, float qx, float qy, float qz, float gain);

// True when the law owns the attitude this cycle; pid.c branches on it.
bool targetGuidanceFcIsActive(void);

// Commanded body rate for an axis, deg/s. Only meaningful while active.
float targetGuidanceFcRateTarget(flight_dynamics_index_t axis);

// Whether the law is driving this axis at all (target_axis_mask), so a masked-off axis
// keeps its normal behaviour instead of being commanded zero.
bool targetGuidanceFcDrivesAxis(flight_dynamics_index_t axis);

// The commanded attitude in RADIANS, for pidTurnAssistant -- which needs the angles the
// law is flying to, not the ones rcCommand would imply.
void targetGuidanceFcAngleTargetsRad(float *bankRad, float *pitchRad);

// The FC's own att_kp x10, reported in MSP 233 so the app's copy cannot drift.
uint16_t targetGuidanceFcAttKp10(void);
