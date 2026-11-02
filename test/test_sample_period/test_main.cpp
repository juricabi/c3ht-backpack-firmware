// Host test for lib/HeadTrackerCore/SamplePeriod.h (measured AHRS time step).
// Run with: pio test -e native -f test_sample_period
#include <unity.h>
#include <math.h>
#include <stdint.h>

#include "SamplePeriod.h"

static uint32_t rngState;
static float urand()   // 0..1
{
    rngState = rngState * 1664525u + 1013904223u;
    return ((rngState >> 8) + 0.5f) / 16777216.0f;
}

void setUp() { rngState = 4242u; }
void tearDown() {}

// IMU runs 2% slow (10.2 ms) with +-150 us IRQ jitter: the estimate converges
// and the integrated time matches wall time.
void test_converges_to_real_rate()
{
    SamplePeriod sp;
    sp.begin(0.01f);
    double t = 0, integrated = 0;
    for (int i = 0; i < 6000; i++)
    {
        t += 0.0102;
        const uint32_t ts = (uint32_t)(t * 1e6 + (urand() - 0.5f) * 300.0f);
        const float dt = sp.update(ts);
        if (i >= 3000) integrated += dt;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.0102f * 0.003f, 0.0102f, sp.period());
    TEST_ASSERT_EQUAL_UINT32(0, sp.missedSamples());
    // last 3000 samples span 30.6 s
    TEST_ASSERT_FLOAT_WITHIN(0.05, 30.6, integrated);
}

// lost samples: the step covers the gap and the loss is counted
void test_missed_samples()
{
    SamplePeriod sp;
    sp.begin(0.01f);
    uint32_t ts = 1000;
    for (int i = 0; i < 100; i++)
    {
        sp.update(ts);
        ts += 10000;
    }
    ts += 10000;            // one sample lost
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.02f, sp.update(ts));
    ts += 30000;            // two more lost
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.03f, sp.update(ts));
    TEST_ASSERT_EQUAL_UINT32(3, sp.missedSamples());
    TEST_ASSERT_EQUAL_UINT32(2, sp.gaps());
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.01f, sp.period());
    // a long outage (IMU re-init) is one normal step, not a jump
    ts += 2000000;
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.01f, sp.update(ts));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.01f, sp.period());
}

// a badly wrong rate is clamped to 0.8..1.2x nominal
void test_clamped()
{
    SamplePeriod sp;
    sp.begin(0.01f);
    uint32_t ts = 0;
    for (int i = 0; i < 5000; i++)
    {
        ts += 14000;        // 1.4x: rounds to n=1
        sp.update(ts);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.012f, sp.period());
    sp.begin(0.01f);
    for (int i = 0; i < 5000; i++)
    {
        ts += 6000;
        sp.update(ts);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.008f, sp.period());
}

// micros() wraps every 71.6 min
void test_wraparound()
{
    SamplePeriod sp;
    sp.begin(0.01f);
    uint32_t ts = 0xFFFFFFFFu - 25000u;
    for (int i = 0; i < 10; i++)
    {
        const float dt = sp.update(ts);
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.01f, dt);
        ts += 10000;
    }
    TEST_ASSERT_EQUAL_UINT32(0, sp.missedSamples());
}

// first sample and a repeated timestamp use the current period
void test_first_and_repeat()
{
    SamplePeriod sp;
    sp.begin(1.0f / 112.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f / 112.1f, sp.update(5));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f / 112.1f, sp.update(5));
    sp.begin(0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, sp.nominal());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_converges_to_real_rate);
    RUN_TEST(test_missed_samples);
    RUN_TEST(test_clamped);
    RUN_TEST(test_wraparound);
    RUN_TEST(test_first_and_repeat);
    return UNITY_END();
}
