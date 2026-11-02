#pragma once

// Gyro bias against die temperature (IMU_SELECTION.md F3).
//
// Every accepted bias estimate (boot calibration or the end of a runtime
// stillness episode) updates one of eight 5 K bins from 15 to 55 degC (an EMA
// of both the bias and the temperature in that bin; temperatures outside the
// range go to the end bins). The filled bins form a piecewise linear curve
// b(T) whose segment slopes are clamped to +-0.05 deg/s/K and which is
// extended linearly past the outermost bins.
//
// The estimator uses only differences of that curve: the bias applied while
// the head is moving is the last accepted estimate plus b(T) - b(T_estimate).
// That keeps the output continuous at the end of every stillness episode
// and lets one bin mean "constant bias" (slope 0). This is what holds pan
// steady while the goggles warm up in flight, when the head is rarely
// still long enough for the runtime estimate.
//
// Plain C++, no Arduino dependency (host test: test/test_gyro_bias).

#include "Fusion.h"

#define BIAS_TEMP_BIN_ALPHA 0.3f        // EMA weight of a new estimate in its bin
#define BIAS_TEMP_MAX_SLOPE 0.05f       // deg/s/K
#define BIAS_TEMP_MIN_SPACING 0.5f      // K between bin temperatures for a slope

class BiasTempModel
{
public:
    static const int BINS = 8;

    BiasTempModel() { reset(); }

    void reset()
    {
        for (int i = 0; i < BINS; i++)
        {
            m_filled[i] = false;
        }
    }

    void add(float tempC, const FusionVector &bias)
    {
        if (!(tempC > -60.0f && tempC < 150.0f))
        {
            return;
        }
        const int i = binOf(tempC);
        if (!m_filled[i])
        {
            m_filled[i] = true;
            m_temp[i] = tempC;
            for (int k = 0; k < 3; k++) m_bias[i][k] = bias.array[k];
            return;
        }
        m_temp[i] += BIAS_TEMP_BIN_ALPHA * (tempC - m_temp[i]);
        for (int k = 0; k < 3; k++) m_bias[i][k] += BIAS_TEMP_BIN_ALPHA * (bias.array[k] - m_bias[i][k]);
    }

    int filledBins() const
    {
        int n = 0;
        for (int i = 0; i < BINS; i++) n += m_filled[i] ? 1 : 0;
        return n;
    }

    // b(t1) - b(t0) per axis
    FusionVector delta(float t0, float t1) const
    {
        FusionVector d;
        for (int k = 0; k < 3; k++)
        {
            d.array[k] = curve(k, t1) - curve(k, t0);
        }
        return d;
    }

    // slope of the curve at t (deg/s/K) per axis
    FusionVector slope(float t) const
    {
        FusionVector s;
        for (int k = 0; k < 3; k++)
        {
            s.array[k] = curve(k, t + 0.5f) - curve(k, t - 0.5f);
        }
        return s;
    }

private:
    static int binOf(float tempC)
    {
        int i = (int)((tempC - 15.0f) / 5.0f);
        if (tempC < 15.0f) i = 0;
        if (i < 0) i = 0;
        if (i >= BINS) i = BINS - 1;
        return i;
    }

    static float clampSlope(float s)
    {
        if (s > BIAS_TEMP_MAX_SLOPE) return BIAS_TEMP_MAX_SLOPE;
        if (s < -BIAS_TEMP_MAX_SLOPE) return -BIAS_TEMP_MAX_SLOPE;
        return s;
    }

    // value of the clamped-slope polyline through the filled bins, axis k,
    // relative to the first filled bin
    float curve(int k, float t) const
    {
        int idx[BINS];
        int m = 0;
        for (int i = 0; i < BINS; i++)
        {
            if (m_filled[i]) idx[m++] = i;
        }
        if (m < 2)
        {
            return 0.0f;                // no bin or one bin: constant
        }
        float slopes[BINS];
        for (int j = 0; j + 1 < m; j++)
        {
            const float dt = m_temp[idx[j + 1]] - m_temp[idx[j]];
            slopes[j] = dt > BIAS_TEMP_MIN_SPACING ? clampSlope((m_bias[idx[j + 1]][k] - m_bias[idx[j]][k]) / dt) : 0.0f;
        }
        float v = 0.0f;
        float tPrev = m_temp[idx[0]];
        if (t <= tPrev)
        {
            return slopes[0] * (t - tPrev);
        }
        for (int j = 0; j + 1 < m; j++)
        {
            const float tNext = m_temp[idx[j + 1]];
            if (t <= tNext || j + 2 == m)
            {
                return v + slopes[j] * (t - tPrev);     // inside, or past the last bin
            }
            v += slopes[j] * (tNext - tPrev);
            tPrev = tNext;
        }
        return v;
    }

    bool m_filled[BINS];
    float m_temp[BINS];
    float m_bias[BINS][3];
};
