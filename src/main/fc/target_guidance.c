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
 * See target_guidance.h for what this law is and why it is shaped this way.
 */

#include <math.h>
#include <string.h>

#include "build/debug.h"
#include "common/maths.h"
#include "common/quaternion.h"

#include "fc/target_guidance.h"

// --- pure kernel ---------------------------------------------------------------------

void targetGuidanceErrorQuat(const fpQuaternion_t *orientation, const fpQuaternion_t *qSp,
                             fpQuaternion_t *qErr)
{
    fpQuaternion_t conj;
    quaternionConjugate(&conj, orientation);
    quaternionMultiply(qErr, &conj, qSp);
    quaternionNormalize(qErr, qErr);

    // q and -q are the same rotation; pick the hemisphere where 2*vec is the SHORT arc.
    if (qErr->q0 < 0.0f) {
        qErr->q0 = -qErr->q0;
        qErr->q1 = -qErr->q1;
        qErr->q2 = -qErr->q2;
        qErr->q3 = -qErr->q3;
    }
}

void targetGuidanceErrorToRate(const fpQuaternion_t *qErr, float attKp, float gain,
                               float rateMaxDps, uint8_t axisMask, float rateDps[3])
{
    const float errDeg[3] = {
        TG_AXIS_SIGN_ROLL  * 2.0f * qErr->q1 * RADIANS_TO_DEGREES(1.0f),
        TG_AXIS_SIGN_PITCH * 2.0f * qErr->q2 * RADIANS_TO_DEGREES(1.0f),
        TG_AXIS_SIGN_YAW   * 2.0f * qErr->q3 * RADIANS_TO_DEGREES(1.0f),
    };
    const uint8_t mask[3] = { TG_AXIS_MASK_ROLL, TG_AXIS_MASK_PITCH, TG_AXIS_MASK_YAW };

    for (int axis = 0; axis < 3; axis++) {
        if (!(axisMask & mask[axis])) {
            rateDps[axis] = 0.0f;
            continue;
        }
        // The form the app assumes: rate = att_kp * gain * err_deg. Keep it exactly, or its
        // own rate cap (gain = min(GAIN, RATE_MAX / (FC_KP * err))) stops meaning anything.
        rateDps[axis] = constrainf(attKp * gain * errDeg[axis], -rateMaxDps, rateMaxDps);
    }
}

void targetGuidanceQuatToEuler(const fpQuaternion_t *q, float *rollDeg, float *pitchDeg, float *yawDeg)
{
    // Same rMat entries as imu.c::imuComputeRotationMatrix / imuUpdateEulerAngles.
    const float r20 = 2.0f * (q->q1 * q->q3 + -(q->q0 * q->q2));
    const float r21 = 2.0f * (q->q2 * q->q3 - -(q->q0 * q->q1));
    const float r22 = 1.0f - 2.0f * q->q1 * q->q1 - 2.0f * q->q2 * q->q2;
    const float r10 = 2.0f * (q->q1 * q->q2 - -(q->q0 * q->q3));
    const float r00 = 1.0f - 2.0f * q->q2 * q->q2 - 2.0f * q->q3 * q->q3;

    if (rollDeg) {
        *rollDeg = RADIANS_TO_DEGREES(atan2_approx(r21, r22));
    }
    if (pitchDeg) {
        *pitchDeg = RADIANS_TO_DEGREES((0.5f * M_PIf) - acos_approx(constrainf(-r20, -1.0f, 1.0f)));
    }
    if (yawDeg) {
        *yawDeg = RADIANS_TO_DEGREES(-atan2_approx(r10, r00));
    }
}

void targetGuidanceQuatFromEuler(float rollDeg, float pitchDeg, float yawDeg, fpQuaternion_t *q)
{
    const float cosRoll  = cos_approx(DEGREES_TO_RADIANS(rollDeg) * 0.5f);
    const float sinRoll  = sin_approx(DEGREES_TO_RADIANS(rollDeg) * 0.5f);
    const float cosPitch = cos_approx(DEGREES_TO_RADIANS(pitchDeg) * 0.5f);
    const float sinPitch = sin_approx(DEGREES_TO_RADIANS(pitchDeg) * 0.5f);
    const float cosYaw   = cos_approx(DEGREES_TO_RADIANS(-yawDeg) * 0.5f);
    const float sinYaw   = sin_approx(DEGREES_TO_RADIANS(-yawDeg) * 0.5f);

    q->q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    q->q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q->q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q->q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
}

void targetGuidanceClampBank(fpQuaternion_t *qSp, float bankMaxDeg)
{
    if (bankMaxDeg <= 0.0f) {
        return;
    }

    float roll, pitch, yaw;
    targetGuidanceQuatToEuler(qSp, &roll, &pitch, &yaw);

    if (fabsf(roll) <= bankMaxDeg) {
        return;
    }

    targetGuidanceQuatFromEuler(constrainf(roll, -bankMaxDeg, bankMaxDeg), pitch, yaw, qSp);
}

void targetGuidanceLevelQuat(const fpQuaternion_t *orientation, float levelTrimPitchDeg,
                             fpQuaternion_t *qOut)
{
    float yaw;
    targetGuidanceQuatToEuler(orientation, NULL, NULL, &yaw);
    targetGuidanceQuatFromEuler(0.0f, -levelTrimPitchDeg, yaw, qOut);
}

void targetGuidanceBlendQuat(const fpQuaternion_t *qFrom, const fpQuaternion_t *qTo, float t,
                             fpQuaternion_t *qOut)
{
    t = constrainf(t, 0.0f, 1.0f);

    fpQuaternion_t to = *qTo;
    const float dot = qFrom->q0 * to.q0 + qFrom->q1 * to.q1 + qFrom->q2 * to.q2 + qFrom->q3 * to.q3;
    if (dot < 0.0f) {  // take the short way round
        to.q0 = -to.q0;
        to.q1 = -to.q1;
        to.q2 = -to.q2;
        to.q3 = -to.q3;
    }

    qOut->q0 = qFrom->q0 + t * (to.q0 - qFrom->q0);
    qOut->q1 = qFrom->q1 + t * (to.q1 - qFrom->q1);
    qOut->q2 = qFrom->q2 + t * (to.q2 - qFrom->q2);
    qOut->q3 = qFrom->q3 + t * (to.q3 - qFrom->q3);
    quaternionNormalize(qOut, qOut);
}

targetGuidanceStage_e targetGuidanceStageForAge(timeDelta_t ageMs, uint16_t staleMs, uint16_t holdMs)
{
    if (ageMs <= (timeDelta_t)staleMs) {
        return TG_STAGE_FRESH;
    }
    if (ageMs <= (timeDelta_t)staleMs + (timeDelta_t)holdMs) {
        return TG_STAGE_HOLD;
    }
    return TG_STAGE_LEVEL_HOLD;
}

int16_t targetGuidancePathMarker(const targetGuidance_t *tg)
{
    if (tg->higherAuthority && tg->armed && tg->boxTarget) {
        return TG_PATH_GATED;
    }
    if (!tg->armed || !tg->boxTarget) {
        return TG_PATH_IDLE;
    }
    if (!tg->latched) {
        return TG_PATH_NO_SETPOINT;
    }

    switch (tg->stage) {
        case TG_STAGE_FRESH:      return TG_PATH_FRESH;
        case TG_STAGE_HOLD:       return TG_PATH_HOLD;
        case TG_STAGE_LEVEL_HOLD: return TG_PATH_LEVEL_HOLD;
        case TG_STAGE_IDLE:
        default:                  return TG_PATH_IDLE;
    }
}

// --- state wrapper -------------------------------------------------------------------

void targetGuidanceInit(targetGuidance_t *tg)
{
    memset(tg, 0, sizeof(*tg));
    quaternionInitUnit(&tg->qSp);
    tg->stage = TG_STAGE_IDLE;
    for (int axis = 0; axis < 3; axis++) {
        pt1FilterReset(&tg->rateFilter[axis], 0.0f);
    }
}

void targetGuidanceConfigure(targetGuidance_t *tg, const targetGuidanceConfig_t *cfg)
{
    tg->cfg = *cfg;
}

void targetGuidanceReceive(targetGuidance_t *tg, timeMs_t nowMs,
                           const fpQuaternion_t *orientation,
                           const fpQuaternion_t *qCorr, float gain)
{
    fpQuaternion_t corr;
    quaternionNormalize(&corr, qCorr);

    // Compose ONCE, here, against the FC's own fresh attitude. Doing this every PID cycle
    // instead would make q_err identically equal to q_corr and open the loop (plan risk 8).
    quaternionMultiply(&tg->qSp, orientation, &corr);
    quaternionNormalize(&tg->qSp, &tg->qSp);
    targetGuidanceClampBank(&tg->qSp, tg->cfg.bank_max_deg);

    tg->gain = constrainf(gain, 0.0f, 1.0f);
    tg->lastUpdateMs = nowMs;
    tg->latched = true;
}

void targetGuidanceSetAuthority(targetGuidance_t *tg, bool armed, bool boxTarget, bool higherAuthority)
{
    tg->armed = armed;
    tg->boxTarget = boxTarget;
    tg->higherAuthority = higherAuthority;
}

bool targetGuidanceIsActive(const targetGuidance_t *tg)
{
    return tg->latched && tg->armed && tg->boxTarget && !tg->higherAuthority;
}

static void publishDebug(const targetGuidance_t *tg, const float rateDps[3], timeMs_t nowMs,
                         float errDeg, timeDelta_t ageMs)
{
    DEBUG_SET(DEBUG_TARGET, 0, lrintf(rateDps[FD_ROLL]));
    DEBUG_SET(DEBUG_TARGET, 1, lrintf(rateDps[FD_PITCH]));
    DEBUG_SET(DEBUG_TARGET, 2, lrintf(rateDps[FD_YAW]));
    DEBUG_SET(DEBUG_TARGET, 3, lrintf(tg->gain * TG_DEBUG_GAIN_SCALE));
    DEBUG_SET(DEBUG_TARGET, 4, lrintf(errDeg * TG_DEBUG_ERR_SCALE));
    // The FC's own clock, in the form the app already decodes, so this log and the blackbox
    // can be put on one time axis afterwards.
    DEBUG_SET(DEBUG_TARGET, 5, (nowMs >> 3) & 0xffff);
    DEBUG_SET(DEBUG_TARGET, 6, ageMs);
    DEBUG_SET(DEBUG_TARGET, 7, targetGuidancePathMarker(tg));
}

targetGuidanceStage_e targetGuidanceUpdate(targetGuidance_t *tg, timeMs_t nowMs,
                                           const fpQuaternion_t *orientation,
                                           float levelTrimPitchDeg, float dT,
                                           float rateDps[3])
{
    rateDps[0] = 0.0f;
    rateDps[1] = 0.0f;
    rateDps[2] = 0.0f;

    if (!targetGuidanceIsActive(tg)) {
        tg->stage = TG_STAGE_IDLE;
        tg->wasActive = false;
        // Still publish: "gated by navigation" and "nothing received yet" are exactly the
        // states the app cannot otherwise tell apart from a silent debug channel.
        publishDebug(tg, rateDps, nowMs, 0.0f, 0);
        return TG_STAGE_IDLE;
    }

    if (!tg->wasActive) {
        // A filter carrying the tail of the previous engagement would put a step into the
        // first cycle of this one.
        for (int axis = 0; axis < 3; axis++) {
            pt1FilterReset(&tg->rateFilter[axis], 0.0f);
        }
        tg->wasActive = true;
    }

    const timeDelta_t ageMs = (timeDelta_t)(nowMs - tg->lastUpdateMs);
    tg->stage = targetGuidanceStageForAge(ageMs, tg->cfg.stale_ms, tg->cfg.hold_ms);

    fpQuaternion_t qTarget, qLevel;
    float gain = tg->gain;

    switch (tg->stage) {
        case TG_STAGE_FRESH:
            qTarget = tg->qSp;
            break;

        case TG_STAGE_HOLD:
            // Hold the latched setpoint, but converge it to level over hold_ms instead of
            // freezing it: a latched hard bank held "as is" costs a low plane its altitude
            // and heading (plan 4.1 p.4). Gain stays the app's — this is still the tail of
            // its engagement, and raising it here would jerk on a single dropped frame.
            targetGuidanceLevelQuat(orientation, levelTrimPitchDeg, &qLevel);
            targetGuidanceBlendQuat(&tg->qSp, &qLevel,
                                    tg->cfg.hold_ms > 0
                                        ? (float)(ageMs - (timeDelta_t)tg->cfg.stale_ms) / (float)tg->cfg.hold_ms
                                        : 1.0f,
                                    &qTarget);
            break;

        case TG_STAGE_LEVEL_HOLD:
        default:
            // Now it is the FC's own law, not the app's, and it has to actually level the
            // aircraft — so full gain, with target_rate_max as the only ceiling.
            targetGuidanceLevelQuat(orientation, levelTrimPitchDeg, &qTarget);
            gain = 1.0f;
            break;
    }

    fpQuaternion_t qErr;
    targetGuidanceErrorQuat(orientation, &qTarget, &qErr);
    targetGuidanceErrorToRate(&qErr, tg->cfg.att_kp, gain, tg->cfg.rate_max_dps,
                              tg->cfg.axis_mask, rateDps);

    // The filter pidLevel used to apply (PID_LEVEL.I, pid.c:717) and that bypassing
    // pidLevel removes. Its comment there is literally our case: the setpoint is computed
    // from attitude at gyro rate, feeding it raw into the rate PID doubles rateError and
    // the D-term amplifies that. It changes the DYNAMICS and leaves rate = kp*gain*err
    // standing, which is why it is the right answer here and lowering att_kp is not.
    if (tg->cfg.lpf_hz > 0.0f && dT > 0.0f) {
        for (int axis = 0; axis < 3; axis++) {
            rateDps[axis] = pt1FilterApply4(&tg->rateFilter[axis], rateDps[axis], tg->cfg.lpf_hz, dT);
        }
    }

    publishDebug(tg, rateDps, nowMs,
                 RADIANS_TO_DEGREES(2.0f * acos_approx(constrainf(qErr.q0, -1.0f, 1.0f))), ageMs);

    return tg->stage;
}
