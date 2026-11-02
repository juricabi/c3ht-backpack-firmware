#pragma once

// Head angle -> PTR channel value.
//
// maxAngleDeg is the angle that gives full scale (outMin at -max, outMax at
// +max); angles beyond it saturate instead of running past the channel range.
// With maxAngleDeg = 180 the result is identical to the original
// map(angle * 100, -18000, 18000, outMin, outMax), because the head tracker
// already normalises its angles to [-180, 180).
//
// Header only and Arduino-free so it can be unit tested on the host
// (test/test_ptr_scale).

// Full-scale angles per axis, whole degrees, 1..180. 180 = original behaviour.
#if !defined(HT_MAX_ANGLE_PAN)
#define HT_MAX_ANGLE_PAN 180
#endif
#if !defined(HT_MAX_ANGLE_TILT)
#define HT_MAX_ANGLE_TILT 180
#endif
#if !defined(HT_MAX_ANGLE_ROLL)
#define HT_MAX_ANGLE_ROLL 180
#endif
#if (HT_MAX_ANGLE_PAN < 1) || (HT_MAX_ANGLE_PAN > 180) || (HT_MAX_ANGLE_TILT < 1) || (HT_MAX_ANGLE_TILT > 180) || (HT_MAX_ANGLE_ROLL < 1) || (HT_MAX_ANGLE_ROLL > 180)
#error "HT_MAX_ANGLE_PAN/TILT/ROLL must be whole degrees in 1..180"
#endif

static inline long ptrAngleToChannel(float angleDeg, long maxAngleDeg, long outMin, long outMax)
{
    if (maxAngleDeg < 1) maxAngleDeg = 1;
    if (maxAngleDeg > 180) maxAngleDeg = 180;
    const float lim = (float)maxAngleDeg;
    if (angleDeg != angleDeg) angleDeg = 0.0f;      // NaN -> centre
    else if (angleDeg > lim) angleDeg = lim;
    else if (angleDeg < -lim) angleDeg = -lim;

    // same integer arithmetic as Arduino map()
    const long inMax = maxAngleDeg * 100;
    const long x = (long)(angleDeg * 100);
    const long delta = x + inMax;
    const long rise = outMax - outMin;
    const long run = 2 * inMax;
    return (delta * rise) / run + outMin;
}
