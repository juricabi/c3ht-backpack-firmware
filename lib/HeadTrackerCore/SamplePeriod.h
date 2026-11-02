#pragma once

// Measured IMU sample period for the AHRS (IMU_SELECTION.md F4, H17).
//
// The data-ready IRQ is timestamped; the interval between consecutive
// samples feeds a slow EMA (alpha 0.01) of the real output data rate,
// clamped to 0.8..1.2x the nominal period, so an IMU whose internal
// oscillator runs a few percent off no longer turns into a yaw scale error.
//
// A gap of more than 1.5 periods means samples were lost (busy loop, WiFi or
// ESP-NOW load). Only the latest sample can be read back, so the step that
// is integrated is n * period with n = round(gap / period): the rotation
// during the lost samples is then applied with the latest rate instead of
// being dropped. Gaps longer than maxSteps periods are treated as a
// discontinuity (one period, e.g. after an IMU re-init) and counted.
//
// Plain C++, no Arduino dependency (host test: test/test_sample_period).

#include <stdint.h>

class SamplePeriod
{
public:
    static const unsigned int maxSteps = 4;

    SamplePeriod() : m_last(0) { begin(0.01f); }

    void begin(float nominalS)
    {
        m_nominal = nominalS > 0.0f ? nominalS : 0.01f;
        m_period = m_nominal;
        m_haveLast = false;
        m_missed = 0;
        m_gaps = 0;
    }

    // tUs: timestamp (micros) of the data-ready IRQ of the sample just read.
    // Returns the time step in seconds to integrate this sample with.
    float update(uint32_t tUs)
    {
        if (!m_haveLast)
        {
            m_haveLast = true;
            m_last = tUs;
            return m_period;
        }
        const float gap = (float)(uint32_t)(tUs - m_last) * 1e-6f;   // wrap-safe
        m_last = tUs;
        if (gap <= 0.0f)
        {
            return m_period;        // same IRQ seen twice: nothing new to time
        }
        unsigned int n = (unsigned int)(gap / m_period + 0.5f);
        if (n < 1)
        {
            n = 1;
        }
        if (gap > 1.5f * m_period)
        {
            m_gaps++;
            m_missed += n - 1;
            if (n > maxSteps)
            {
                return m_period;    // discontinuity, not a short loss
            }
        }
        const float alpha = 0.01f;
        const float per = gap / (float)n;
        m_period += alpha * (per - m_period);
        if (m_period < 0.8f * m_nominal) m_period = 0.8f * m_nominal;
        if (m_period > 1.2f * m_nominal) m_period = 1.2f * m_nominal;
        return (float)n * m_period;
    }

    float period() const { return m_period; }
    float nominal() const { return m_nominal; }
    uint32_t missedSamples() const { return m_missed; }
    uint32_t gaps() const { return m_gaps; }

private:
    float m_nominal;
    float m_period;
    bool m_haveLast;
    uint32_t m_last;
    uint32_t m_missed;
    uint32_t m_gaps;
};
