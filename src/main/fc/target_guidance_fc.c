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
 * See target_guidance_fc.h for why the globals live here and the law does not.
 */

#include <stdbool.h>

#include "platform.h"

#include "common/maths.h"
#include "common/quaternion.h"

#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/target_guidance.h"
#include "fc/target_guidance_fc.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"

#include "navigation/navigation.h"

static EXTENDED_FASTRAM targetGuidance_t targetGuidanceState;
static EXTENDED_FASTRAM float targetGuidanceRateDps[3];

static void refreshConfig(void)
{
    targetGuidanceConfig_t cfg;

    cfg.att_kp = pidProfile()->target_att_kp / 10.0f;
    cfg.rate_max_dps = pidProfile()->target_rate_max;
    // 0 means "keep the filter the airframe is already tuned with": pidLevel's cutoff is
    // the profile's LEVEL I term, and bypassing pidLevel must not silently drop it. A zero
    // LEVEL I then means no filter, exactly as in pidLevel (`if (pidBank()->pid[PID_LEVEL].I)`).
    cfg.lpf_hz = pidProfile()->target_lpf_hz > 0
                     ? (float)pidProfile()->target_lpf_hz
                     : (float)pidBank()->pid[PID_LEVEL].I;
    cfg.stale_ms = pidProfile()->target_stale_ms;
    cfg.hold_ms = pidProfile()->target_hold_ms;
    cfg.bank_max_deg = pidProfile()->target_bank_max;
    cfg.axis_mask = pidProfile()->target_axis_mask;

    targetGuidanceConfigure(&targetGuidanceState, &cfg);
}

void targetGuidanceFcReceive(float qw, float qx, float qy, float qz, float gain)
{
    fpQuaternion_t qCorr = { .q0 = qw, .q1 = qx, .q2 = qy, .q3 = qz };

    // The settings have to be current BEFORE the setpoint is latched: the bank clamp is
    // applied here, at receipt, not on the way out.
    refreshConfig();
    targetGuidanceReceive(&targetGuidanceState, millis(), &orientation, &qCorr, gain);
}

void targetGuidanceFcUpdate(timeUs_t currentTimeUs, float dT)
{
    UNUSED(currentTimeUs);

    refreshConfig();

    // Authority is recomputed from scratch every cycle, never set by an event, so the law
    // cannot get stuck active if something upstream wedges.
    //
    // Failsafe and fixed-wing navigation command attitude through the very angle path this
    // law bypasses (navigation_fixedwing.c writes angles into rcCommand), so whenever either
    // of them wants the aircraft, guidance lets go. Without this, an engaged target mode
    // silently overrides RTH and autoland -- and in a log that reads as guidance working
    // normally right up to the crash.
    const bool higherAuthority = FLIGHT_MODE(FAILSAFE_MODE)
                                 || failsafeRequiresAngleMode()
                                 || navigationRequiresAngleMode();

    targetGuidanceSetAuthority(&targetGuidanceState,
                               ARMING_FLAG(ARMED) && STATE(AIRPLANE),
                               IS_RC_MODE_ACTIVE(BOXTARGET),
                               higherAuthority);

    targetGuidanceUpdate(&targetGuidanceState, millis(), &orientation,
                         getFixedWingLevelTrim(), dT, targetGuidanceRateDps);
}

bool targetGuidanceFcIsActive(void)
{
    return targetGuidanceIsActive(&targetGuidanceState);
}

float targetGuidanceFcRateTarget(flight_dynamics_index_t axis)
{
    return targetGuidanceRateDps[axis];
}

bool targetGuidanceFcDrivesAxis(flight_dynamics_index_t axis)
{
    const uint8_t mask = targetGuidanceState.cfg.axis_mask;

    switch (axis) {
        case FD_ROLL:  return (mask & TG_AXIS_MASK_ROLL) != 0;
        case FD_PITCH: return (mask & TG_AXIS_MASK_PITCH) != 0;
        case FD_YAW:   return (mask & TG_AXIS_MASK_YAW) != 0;
        default:       return false;
    }
}

void targetGuidanceFcAngleTargetsRad(float *bankRad, float *pitchRad)
{
    *bankRad = DEGREES_TO_RADIANS(targetGuidanceState.targetRollDeg);
    *pitchRad = DEGREES_TO_RADIANS(targetGuidanceState.targetPitchDeg);
}

uint16_t targetGuidanceFcAttKp10(void)
{
    return pidProfile()->target_att_kp;
}
