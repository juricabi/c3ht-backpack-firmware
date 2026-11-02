// Host test for lib/HeadTrackerCore/GyroBias: synthetic 100 Hz gyro data with a
// constant zero-rate offset, a slow temperature drift and white noise.
// Run with: pio test -e native -f test_gyro_bias
#include <unity.h>
#include <math.h>
#include <stdint.h>

#include "GyroBias.h"

static const unsigned int RATE = 100;
static const float DT = 1.0f / RATE;
static const float PI_F = 3.14159265f;

// deterministic Gaussian noise (LCG + Box-Muller)
static uint32_t rngState;
static float urand()
{
    rngState = rngState * 1664525u + 1013904223u;
    return ((rngState >> 8) + 0.5f) / 16777216.0f;
}
static float gauss(float sigma)
{
    const float u1 = urand(), u2 = urand();
    return sigma * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI_F * u2);
}

struct Sim
{
    float bias[3];          // true offset (deg/s)
    float driftPerMin[3];   // true offset change (deg/s per minute), e.g. warm-up
    float noise;            // white noise sigma (deg/s)
};

// head motion: returns the true angular rate (deg/s) at time t
typedef void (*MotionFn)(float t, float out[3]);

static void still(float, float out[3]) { out[0] = out[1] = out[2] = 0; }

// a pilot: 20 s of looking around (+-40 deg/s), then 10 s of small head
// movement (0.3 deg/s tremor), repeated
static void pilot(float t, float out[3])
{
    const float phase = fmodf(t, 30.0f);
    if (phase < 20.0f)
    {
        out[0] = 15.0f * sinf(2 * PI_F * 0.31f * t);
        out[1] = 10.0f * sinf(2 * PI_F * 0.17f * t + 1.0f);
        out[2] = 40.0f * sinf(2 * PI_F * 0.23f * t + 2.0f);
    }
    else
    {
        out[0] = 0.3f * sinf(2 * PI_F * 1.3f * t);
        out[1] = 0.3f * sinf(2 * PI_F * 0.9f * t);
        out[2] = 0.3f * sinf(2 * PI_F * 1.1f * t);
    }
}

// keeps moving for the first 20 s (boot sample must time out), then still
static void busyBoot(float t, float out[3])
{
    if (t < 20.0f)
    {
        out[0] = 20.0f * sinf(2 * PI_F * 0.5f * t);
        out[1] = 20.0f * sinf(2 * PI_F * 0.4f * t);
        out[2] = 20.0f * sinf(2 * PI_F * 0.3f * t);
    }
    else
    {
        still(t, out);
    }
}

struct Result
{
    float residual[3];      // |true bias - estimate| at the end (deg/s)
    float yawErrLastMin;    // integrated yaw error over the last 60 s (deg)
    unsigned int bootDoneAt; // sample index at which the boot phase ended
};

static Result run(const Sim &sim, MotionFn motion, float seconds, GyroBiasEstimator &est)
{
    Result r = {};
    r.bootDoneAt = 0;
    const unsigned int n = (unsigned int)(seconds * RATE);
    const unsigned int lastMin = n - 60 * RATE;
    float trueBias[3] = {0, 0, 0};
    for (unsigned int i = 0; i < n; i++)
    {
        const float t = i * DT;
        float w[3];
        motion(t, w);
        FusionVector g;
        for (int k = 0; k < 3; k++)
        {
            trueBias[k] = sim.bias[k] + sim.driftPerMin[k] * t / 60.0f;
            g.array[k] = w[k] + trueBias[k] + gauss(sim.noise);
        }
        const bool wasDone = est.bootSampleDone();
        const FusionVector c = est.update(g);
        if (!wasDone && est.bootSampleDone())
        {
            r.bootDoneAt = i + 1;
        }
        if (i >= lastMin)
        {
            r.yawErrLastMin += (c.axis.z - w[2]) * DT;
        }
    }
    const FusionVector b = est.bias();
    for (int k = 0; k < 3; k++)
    {
        r.residual[k] = fabsf(trueBias[k] - b.array[k]);
    }
    return r;
}

static const float MAX_DRIFT_DEG_PER_MIN = 2.0f;
static const float MAX_RESIDUAL_DPS = MAX_DRIFT_DEG_PER_MIN / 60.0f;

void setUp() { rngState = 12345u; }
void tearDown() {}

// The requirement: constant offset + noise, still board, drift after 10 min < 2 deg/min.
void test_still_constant_offset_10min()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{1.2f, -0.7f, 1.8f}, {0, 0, 0}, 0.1f};
    const Result r = run(sim, still, 600.0f, est);
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_LESS_THAN_FLOAT(MAX_RESIDUAL_DPS, r.residual[k]);
    }
    TEST_ASSERT_LESS_THAN_FLOAT(MAX_DRIFT_DEG_PER_MIN, fabsf(r.yawErrLastMin));
}

// Same with a moving pilot and a warm-up drift of 0.15 deg/s over 10 min on yaw
// (about the ICM-42670-P ZRO tempco for a 10 degC rise).
void test_pilot_motion_with_warmup_10min()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{0.9f, -1.1f, 1.5f}, {0.005f, -0.005f, 0.015f}, 0.1f};
    const Result r = run(sim, pilot, 600.0f, est);
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_LESS_THAN_FLOAT(MAX_RESIDUAL_DPS, r.residual[k]);
    }
    TEST_ASSERT_LESS_THAN_FLOAT(MAX_DRIFT_DEG_PER_MIN, fabsf(r.yawErrLastMin));
}

// Without estimation the same data drifts by 90 deg/min: the test can fail.
void test_reference_without_estimation_drifts()
{
    const Sim sim = {{1.2f, -0.7f, 1.5f}, {0, 0, 0}, 0.1f};
    float yaw = 0;
    for (unsigned int i = 0; i < 60 * RATE; i++)
    {
        yaw += (sim.bias[2] + gauss(sim.noise)) * DT;
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(MAX_DRIFT_DEG_PER_MIN, fabsf(yaw));
}

// Boot sample: still from power-on -> accepted after the 2 s window, already accurate.
void test_boot_sample_still()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{1.2f, -0.7f, 1.8f}, {0, 0, 0}, 0.1f};
    const Result r = run(sim, still, 3.0f + 60.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleAccepted());
    TEST_ASSERT_EQUAL_UINT(2 * RATE, r.bootDoneAt);

    GyroBiasEstimator est2;
    est2.begin(GyroBiasDefaultSettings(RATE));
    rngState = 999u;
    for (unsigned int i = 0; i < 2 * RATE; i++)
    {
        FusionVector g = {{sim.bias[0] + gauss(0.1f), sim.bias[1] + gauss(0.1f), sim.bias[2] + gauss(0.1f)}};
        est2.update(g);
    }
    TEST_ASSERT_TRUE(est2.bootSampleAccepted());
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_FLOAT_WITHIN(0.05f, sim.bias[k], est2.bias().array[k]);
    }
}

// Boot sample: board keeps moving -> gives up after the 15 s timeout, runtime
// estimation still converges once the board is still.
void test_boot_sample_timeout()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{1.0f, 1.0f, -1.0f}, {0, 0, 0}, 0.1f};
    const Result r = run(sim, busyBoot, 120.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleDone());
    TEST_ASSERT_FALSE(est.bootSampleAccepted());
    TEST_ASSERT_EQUAL_UINT(15 * RATE, r.bootDoneAt);
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_LESS_THAN_FLOAT(MAX_RESIDUAL_DPS, r.residual[k]);
    }
}

// A slow constant turn during the boot window is "still" for the p-p test; the
// clamp bounds the damage, and the estimate never exceeds it.
void test_clamp()
{
    GyroBiasEstimator est;
    GyroBiasSettings s = GyroBiasDefaultSettings(RATE);
    est.begin(s);
    const Sim sim = {{8.0f, -8.0f, 6.0f}, {0, 0, 0}, 0.1f};
    run(sim, still, 120.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleAccepted());
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_TRUE(fabsf(est.bias().array[k]) <= s.maxBiasDps);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, s.maxBiasDps, fabsf(est.bias().array[k]));
    }
}

// begin() resets everything (used after a new web-UI calibration).
void test_begin_resets()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{1.0f, 1.0f, 1.0f}, {0, 0, 0}, 0.1f};
    run(sim, still, 63.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleAccepted());
    est.begin(GyroBiasDefaultSettings(RATE));
    TEST_ASSERT_FALSE(est.bootSampleDone());
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, est.bias().array[k]);
    }
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_still_constant_offset_10min);
    RUN_TEST(test_pilot_motion_with_warmup_10min);
    RUN_TEST(test_reference_without_estimation_drifts);
    RUN_TEST(test_boot_sample_still);
    RUN_TEST(test_boot_sample_timeout);
    RUN_TEST(test_clamp);
    RUN_TEST(test_begin_resets);
    return UNITY_END();
}
