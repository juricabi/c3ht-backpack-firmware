// Host test for lib/HeadTrackerCore/GyroBias: synthetic 100 Hz IMU data with
// a constant zero-rate offset, an optional temperature-driven drift, white
// noise on gyro and accel, and simulated head motion (the accel is the
// gravity vector rotated by the integrated true motion).
// Run with: pio test -e native -f test_gyro_bias
#include <unity.h>
#include <math.h>
#include <stdint.h>

#include "GyroBias.h"

static const unsigned int RATE = 100;
static const float DT = 1.0f / RATE;
static const float PI_F = 3.14159265f;
static const float D2R = PI_F / 180.0f;

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

// temperature profile: returns degC at time t
typedef float (*TempFn)(float t);
static float temp25(float) { return 25.0f; }
static float noTemp(float) { return NAN; }

struct Sim
{
    float bias[3];          // true offset at 25 degC (deg/s)
    float tempco[3];        // true offset change (deg/s per K)
    float noise;            // gyro white noise sigma (deg/s)
    TempFn temp;
    bool hideTemp;          // the bias follows temp, but the estimator gets NAN
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

// keeps moving for the first 20 s (boot calibration must time out), then still
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

// still for 10 s (boot), then a slow, steady 0.5 deg/s pan about gravity
static void slowPan(float t, float out[3])
{
    out[0] = out[1] = 0;
    out[2] = t < 10.0f ? 0.0f : 0.5f;
}

// still for 10 s (boot), then a slow, steady 0.5 deg/s pitch (tilts gravity)
static void slowPitch(float t, float out[3])
{
    out[1] = out[2] = 0;
    out[0] = t < 10.0f ? 0.0f : 0.5f;
}

struct Result
{
    float residual[3];      // |true bias - estimate| at the end (deg/s)
    float maxAbsBias;       // largest |estimate| on any axis at any time
    float yawErrLastMin;    // integrated yaw error over the last 60 s (deg)
    unsigned int bootDoneAt; // sample index at which the boot phase ended
};

struct State
{
    float t;
    float grav[3];          // gravity in the body frame (g)
};

static void stateInit(State &s)
{
    s.t = 0;
    s.grav[0] = 0;
    s.grav[1] = 0;
    s.grav[2] = 1;
}

static Result run(const Sim &sim, MotionFn motion, float seconds, GyroBiasEstimator &est, State &st)
{
    Result r = {};
    r.bootDoneAt = 0;
    const unsigned int n = (unsigned int)(seconds * RATE + 0.5f);
    const unsigned int lastMin = n > 60 * RATE ? n - 60 * RATE : 0;
    float trueBias[3] = {0, 0, 0};
    for (unsigned int i = 0; i < n; i++)
    {
        const float t = st.t;
        float w[3];
        motion(t, w);
        const float T = sim.temp(t);
        const float Tb = isnan(T) ? 25.0f : T;
        FusionVector g, a;
        for (int k = 0; k < 3; k++)
        {
            trueBias[k] = sim.bias[k] + sim.tempco[k] * (Tb - 25.0f);
            g.array[k] = w[k] + trueBias[k] + gauss(sim.noise);
            a.array[k] = st.grav[k] + gauss(0.001f);
        }
        const bool wasDone = est.bootSampleDone();
        const float Tfeed = (sim.hideTemp || isnan(T)) ? NAN : T + gauss(0.02f);
        const FusionVector c = est.update(g, a, Tfeed);
        if (!wasDone && est.bootSampleDone())
        {
            r.bootDoneAt = i + 1;
        }
        if (i >= lastMin)
        {
            r.yawErrLastMin += (c.axis.z - w[2]) * DT;
        }
        for (int k = 0; k < 3; k++)
        {
            const float b = fabsf(est.bias().array[k]);
            if (b > r.maxAbsBias) r.maxAbsBias = b;
        }
        // rotate the body-frame gravity: dv/dt = v x w
        float v[3] = {st.grav[0], st.grav[1], st.grav[2]};
        const float wr[3] = {w[0] * D2R, w[1] * D2R, w[2] * D2R};
        st.grav[0] += (v[1] * wr[2] - v[2] * wr[1]) * DT;
        st.grav[1] += (v[2] * wr[0] - v[0] * wr[2]) * DT;
        st.grav[2] += (v[0] * wr[1] - v[1] * wr[0]) * DT;
        const float nn = sqrtf(st.grav[0] * st.grav[0] + st.grav[1] * st.grav[1] + st.grav[2] * st.grav[2]);
        for (int k = 0; k < 3; k++) st.grav[k] /= nn;
        st.t += DT;
    }
    const FusionVector b = est.bias();
    for (int k = 0; k < 3; k++)
    {
        r.residual[k] = fabsf(trueBias[k] - b.array[k]);
    }
    return r;
}

static Result run(const Sim &sim, MotionFn motion, float seconds, GyroBiasEstimator &est)
{
    State st;
    stateInit(st);
    return run(sim, motion, seconds, est, st);
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
    const Sim sim = {{1.2f, -0.7f, 1.8f}, {0, 0, 0}, 0.07f, temp25};
    const Result r = run(sim, still, 600.0f, est);
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_LESS_THAN_FLOAT(MAX_RESIDUAL_DPS, r.residual[k]);
    }
    TEST_ASSERT_LESS_THAN_FLOAT(MAX_DRIFT_DEG_PER_MIN, fabsf(r.yawErrLastMin));
}

// Same without a temperature sensor (QMI8658C)
void test_still_constant_offset_no_temperature()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{1.2f, -0.7f, 1.8f}, {0, 0, 0}, 0.07f, noTemp};
    const Result r = run(sim, still, 600.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleAccepted());
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_LESS_THAN_FLOAT(MAX_RESIDUAL_DPS, r.residual[k]);
    }
}

// warm-up: 25 -> 35 degC over 10 min, i.e. 0.15 deg/s of yaw offset change at
// the ICM-42670-P tempco (0.015 deg/s/K)
static float warmup10min(float t) { return 25.0f + 10.0f * (t > 600.0f ? 1.0f : t / 600.0f); }

// Moving pilot (still only in short tremor windows) with the warm-up drift.
// The pilot is moving at power-on, so the boot calibration times out and the
// runtime estimate has to find the full offset by itself.
void test_pilot_motion_with_warmup_10min()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{0.9f, -1.1f, 1.5f}, {0.005f, -0.005f, 0.015f}, 0.07f, warmup10min};
    const Result r = run(sim, pilot, 600.0f, est);
    TEST_ASSERT_FALSE(est.bootSampleAccepted());
    TEST_ASSERT_TRUE(est.locked());
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_LESS_THAN_FLOAT(MAX_RESIDUAL_DPS, r.residual[k]);
    }
    TEST_ASSERT_LESS_THAN_FLOAT(MAX_DRIFT_DEG_PER_MIN, fabsf(r.yawErrLastMin));
}

// Without estimation the same data drifts by 90 deg/min: the test can fail.
void test_reference_without_estimation_drifts()
{
    const Sim sim = {{1.2f, -0.7f, 1.5f}, {0, 0, 0}, 0.07f, temp25};
    float yaw = 0;
    for (unsigned int i = 0; i < 60 * RATE; i++)
    {
        yaw += (sim.bias[2] + gauss(sim.noise)) * DT;
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(MAX_DRIFT_DEG_PER_MIN, fabsf(yaw));
}

// F1: still from power-on -> 0.2 s discard + 1 s gate + 2 s mean, accurate.
void test_boot_calibration_still()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{1.2f, -0.7f, 1.8f}, {0, 0, 0}, 0.07f, temp25};
    const Result r = run(sim, still, 3.3f, est);
    TEST_ASSERT_TRUE(est.bootSampleAccepted());
    TEST_ASSERT_TRUE(est.locked());
    TEST_ASSERT_EQUAL_UINT(20 + 100 + 200, r.bootDoneAt);
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_FLOAT_WITHIN(0.02f, sim.bias[k], est.bias().array[k]);
    }
}

// F1: board keeps moving -> gives up 10 s after the discard; the runtime
// estimate (unlocked: 3 deg/s threshold, no step clamp) still converges.
void test_boot_calibration_timeout()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{1.0f, 1.0f, -1.0f}, {0, 0, 0}, 0.07f, temp25};
    const Result r = run(sim, busyBoot, 120.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleDone());
    TEST_ASSERT_FALSE(est.bootSampleAccepted());
    TEST_ASSERT_EQUAL_UINT(20 + 1000, r.bootDoneAt);
    TEST_ASSERT_TRUE(est.locked());
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_LESS_THAN_FLOAT(MAX_RESIDUAL_DPS, r.residual[k]);
    }
}

// F1: a noisy (vibrating) board fails the 0.15 deg/s standard deviation gate
void test_boot_calibration_rejects_noise()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{1.0f, 1.0f, -1.0f}, {0, 0, 0}, 0.3f, temp25};
    const Result r = run(sim, still, 11.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleDone());
    TEST_ASSERT_FALSE(est.bootSampleAccepted());
    TEST_ASSERT_EQUAL_UINT(20 + 1000, r.bootDoneAt);
}

// F1 sanity check: a still board with a 4 deg/s offset is rejected (above the
// 3 deg/s limit), and the runtime estimate never sees "still" either (it is
// above the 3 deg/s threshold), so the stored calibration is kept as is.
void test_boot_calibration_rejects_large_bias()
{
    GyroBiasEstimator est;
    const GyroBiasSettings s = GyroBiasDefaultSettings(RATE);
    est.begin(s);
    const Sim sim = {{4.0f, -0.5f, 0.5f}, {0, 0, 0}, 0.07f, temp25};
    const Result r = run(sim, still, 120.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleDone());
    TEST_ASSERT_FALSE(est.bootSampleAccepted());
    TEST_ASSERT_EQUAL_UINT(20 + 100 + 200, r.bootDoneAt);
    TEST_ASSERT_FALSE(est.locked());
    TEST_ASSERT_TRUE(r.maxAbsBias <= s.maxBiasDps);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, est.bias().array[0]);
}

// F2 step clamp: after the boot calibration, a slow steady pan about gravity
// (0.5 deg/s, below the 1 deg/s threshold, accel unchanged) looks exactly
// like bias. One episode may absorb at most 0.05 deg/s of it.
void test_step_clamp_limits_slow_pan()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{0.5f, 0.3f, -0.4f}, {0, 0, 0}, 0.07f, temp25};
    const Result r = run(sim, slowPan, 70.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleAccepted());
    TEST_ASSERT_TRUE(r.residual[2] < 0.05f + 0.02f);
    // without the clamp the 0.02 Hz low-pass would have absorbed ~0.5 deg/s
    TEST_ASSERT_TRUE(r.residual[2] > 0.03f);
}

// F2 accel gate: a slow pitch (0.5 deg/s, below the gyro threshold) turns the
// gravity vector, so it is not taken as stillness.
// Repeated slow pans (body or chair yaw, slowly following a target) with a
// quick snap back in between. Each pan is a stillness episode that may move
// the bias by one step clamp; the total must stay within the envelope around
// the boot calibration instead of growing by one step per episode (it used
// to reach the pan rate itself: -90 deg after 40 cycles of 0.3 deg/s x 8 s).
static float slowPanCycles(GyroBiasEstimator &est, float rate, float panS, int cycles, float &yawErr)
{
    const FusionVector a = {{0.0f, 0.0f, 1.0f}};
    const float trueBias = 0.4f;
    double err = 0;
    for (int i = 0; i < 10 * (int)RATE; i++)     // boot: still
    {
        const FusionVector g = {{0.0f, 0.0f, trueBias + gauss(0.07f)}};
        err += est.update(g, a, 30.0f).axis.z * DT;
    }
    float maxDev = 0;
    const int nPan = (int)(panS * RATE + 0.5f);
    const int nBack = (int)(0.2f * RATE);
    const float back = -rate * panS / 0.2f;
    for (int c = 0; c < cycles; c++)
    {
        for (int i = 0; i < nPan + nBack; i++)
        {
            const float w = i < nPan ? rate : back;
            const FusionVector g = {{0.0f, 0.0f, w + trueBias + gauss(0.07f)}};
            err += (est.update(g, a, 30.0f).axis.z - w) * DT;
            const float dev = fabsf(est.bias().axis.z - trueBias);
            if (dev > maxDev) maxDev = dev;
        }
    }
    yawErr = (float)err;
    return maxDev;
}

void test_repeated_slow_pans_stay_in_envelope()
{
    const float rates[3] = {0.3f, 0.8f, 0.3f};
    const float pans[3] = {8.0f, 6.0f, 3.5f};
    for (int k = 0; k < 3; k++)
    {
        rngState = 12345u + k;
        GyroBiasEstimator est;
        est.begin(GyroBiasDefaultSettings(RATE));
        float yawErr = 0;
        const float maxDev = slowPanCycles(est, rates[k], pans[k], 40, yawErr);
        TEST_ASSERT_TRUE(est.bootSampleAccepted());
        // 0.05 deg/s envelope (constant temperature) + boot calibration error
        TEST_ASSERT_LESS_THAN_FLOAT(0.05f + 0.01f, maxDev);
        // at most ~0.06 deg/s over the ~5.5 min of cycles
        TEST_ASSERT_LESS_THAN_FLOAT(0.06f * 40 * (pans[k] + 0.2f), fabsf(yawErr));
    }
}

void test_accel_gate_blocks_slow_tilt()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{0.5f, 0.3f, -0.4f}, {0, 0, 0}, 0.07f, temp25};
    const Result r = run(sim, slowPitch, 70.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleAccepted());
    // it stops adapting within the first second of the tilt (0.5 deg) and
    // never starts again; that first second is bounded by the step clamp
    TEST_ASSERT_FALSE(est.adapting());
    TEST_ASSERT_LESS_THAN_FLOAT(0.05f + 0.02f, r.residual[0]);
}

// begin() resets everything; restartCalibrated() means "residual bias is 0"
void test_begin_and_restart_calibrated()
{
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Sim sim = {{1.0f, 1.0f, 1.0f}, {0, 0, 0}, 0.07f, temp25};
    run(sim, still, 10.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleAccepted());
    est.begin(GyroBiasDefaultSettings(RATE));
    TEST_ASSERT_FALSE(est.bootSampleDone());
    TEST_ASSERT_FALSE(est.locked());
    for (int k = 0; k < 3; k++)
    {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, est.bias().array[k]);
    }
    est.restartCalibrated(30.0f);
    TEST_ASSERT_TRUE(est.bootSampleDone());
    TEST_ASSERT_TRUE(est.locked());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 30.0f, est.temperature());
}

// StillCalibrator on its own (used by the web "ci" command)
void test_still_calibrator()
{
    GyroBiasSettings s = GyroBiasDefaultSettings(RATE);
    s.maxBootBiasDps = 30.0f;
    StillCalibrator cal;
    cal.begin(s);
    StillCalibrator::Result res = StillCalibrator::RUNNING;
    unsigned int i = 0;
    while (res == StillCalibrator::RUNNING && i < 2000)
    {
        FusionVector g = {{7.5f + gauss(0.07f), -2.0f + gauss(0.07f), 0.3f + gauss(0.07f)}};
        FusionVector a = {{0.0f + gauss(0.001f), 0.0f + gauss(0.001f), 1.0f + gauss(0.001f)}};
        res = cal.update(g, a);
        i++;
    }
    TEST_ASSERT_EQUAL_INT(StillCalibrator::ACCEPTED, res);
    TEST_ASSERT_EQUAL_UINT(320, i);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 7.5f, cal.mean().axis.x);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, -2.0f, cal.mean().axis.y);

    // an in-spec accel can read 1 g +- ~0.04 g at rest (+-25 mg zero-g offset,
    // +-1 % sensitivity): still accepted
    cal.begin(s);
    res = StillCalibrator::RUNNING;
    for (i = 0; i < 2000 && res == StillCalibrator::RUNNING; i++)
    {
        FusionVector g = {{gauss(0.07f), gauss(0.07f), gauss(0.07f)}};
        FusionVector a = {{0.02f + gauss(0.001f), -0.015f + gauss(0.001f), 1.035f + gauss(0.001f)}};
        res = cal.update(g, a);
    }
    TEST_ASSERT_EQUAL_INT(StillCalibrator::ACCEPTED, res);

    // a reading far from 1 g (broken accel, free fall, being shaken)
    cal.begin(s);
    res = StillCalibrator::RUNNING;
    for (i = 0; i < 2000 && res == StillCalibrator::RUNNING; i++)
    {
        FusionVector g = {{gauss(0.07f), gauss(0.07f), gauss(0.07f)}};
        FusionVector a = {{0.0f, 0.0f, 1.2f}};
        res = cal.update(g, a);
    }
    TEST_ASSERT_EQUAL_INT(StillCalibrator::TIMED_OUT, res);

    // linear bumps: the gyro is quiet but the accel norm jumps by 0.05 g
    // every 0.5 s (walking with the goggles in hand): never a 1 s window
    cal.begin(s);
    res = StillCalibrator::RUNNING;
    for (i = 0; i < 2000 && res == StillCalibrator::RUNNING; i++)
    {
        FusionVector g = {{gauss(0.07f), gauss(0.07f), gauss(0.07f)}};
        FusionVector a = {{0.0f, 0.0f, (i % 50) == 25 ? 1.05f : 1.0f}};
        res = cal.update(g, a);
    }
    TEST_ASSERT_EQUAL_INT(StillCalibrator::TIMED_OUT, res);
}

// F3 scenario: still at 25 degC (boot), 5 min of continuous motion while the
// goggles warm to 35 degC, 1 min still at 35 degC, then 5 min of continuous
// motion while they warm to 40 degC. In the last phase nothing is still, so
// only the learned bias-vs-temperature curve can follow the yaw offset
// (0.015 deg/s/K * 5 K = 0.075 deg/s, 4.5 deg/min).
static float f3Temp(float t)
{
    if (t < 30.0f) return 25.0f;
    if (t < 330.0f) return 25.0f + 10.0f * (t - 30.0f) / 300.0f;
    if (t < 390.0f) return 35.0f;
    if (t < 690.0f) return 35.0f + 5.0f * (t - 390.0f) / 300.0f;
    return 40.0f;
}
static void f3Motion(float t, float out[3])
{
    if (t < 30.0f || (t >= 330.0f && t < 390.0f))
    {
        still(t, out);
        return;
    }
    out[0] = 20.0f * sinf(2 * PI_F * 0.21f * t);
    out[1] = 15.0f * sinf(2 * PI_F * 0.13f * t + 1.0f);
    out[2] = 30.0f * sinf(2 * PI_F * 0.17f * t + 2.0f);
}

void test_temperature_compensation_warmup()
{
    const Sim sim = {{0.8f, -0.6f, 1.1f}, {0.005f, -0.008f, 0.015f}, 0.07f, f3Temp};
    GyroBiasEstimator est;
    est.begin(GyroBiasDefaultSettings(RATE));
    const Result r = run(sim, f3Motion, 690.0f, est);
    TEST_ASSERT_TRUE(est.bootSampleAccepted());
    TEST_ASSERT_EQUAL_INT(2, est.temperatureModel().filledBins());
    TEST_ASSERT_FLOAT_WITHIN(0.003f, 0.015f, est.temperatureModel().slope(37.5f).axis.z);

    // the same data without a temperature reading (F2 alone)
    Sim simNoT = sim;
    simNoT.hideTemp = true;
    GyroBiasEstimator ref;
    ref.begin(GyroBiasDefaultSettings(RATE));
    rngState = 12345u;
    const Result rr = run(simNoT, f3Motion, 690.0f, ref);
    const float refResidual = rr.residual[2];

    // F2 alone is left 0.075 deg/s off; F3 must remove at least 70% of it
    TEST_ASSERT_GREATER_THAN_FLOAT(0.06f, refResidual);
    TEST_ASSERT_LESS_THAN_FLOAT(0.3f * 0.075f, r.residual[2]);
    TEST_ASSERT_LESS_THAN_FLOAT(0.3f * 0.04f, r.residual[1]);
}

void test_temperature_model()
{
    BiasTempModel m;
    FusionVector b = {{1.0f, 2.0f, 3.0f}};
    TEST_ASSERT_EQUAL_FLOAT(0.0f, m.delta(20.0f, 40.0f).axis.x);     // empty
    m.add(25.0f, b);
    TEST_ASSERT_EQUAL_INT(1, m.filledBins());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, m.delta(20.0f, 40.0f).axis.z);     // one bin: constant
    FusionVector b2 = {{1.1f, 2.0f, 4.0f}};
    m.add(35.0f, b2);
    TEST_ASSERT_EQUAL_INT(2, m.filledBins());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.1f, m.delta(25.0f, 35.0f).axis.x);    // 0.01/K
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.05f, m.delta(25.0f, 30.0f).axis.x);   // interpolated
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.15f, m.delta(25.0f, 40.0f).axis.x);   // extended past the last bin
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.05f, m.delta(25.0f, 20.0f).axis.x);  // and below the first
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, m.delta(25.0f, 35.0f).axis.z);    // 0.1/K clamped to 0.05/K
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, m.delta(25.0f, 35.0f).axis.y);
    // EMA in a bin
    FusionVector b3 = {{1.2f, 2.0f, 4.0f}};
    m.add(36.0f, b3);
    TEST_ASSERT_EQUAL_INT(2, m.filledBins());
    // two estimates closer than 0.5 K in adjacent bins give no slope
    BiasTempModel m2;
    m2.add(29.9f, b);
    m2.add(30.1f, b2);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, m2.delta(20.0f, 40.0f).axis.x);
    // nonsense temperature is ignored
    m2.add(NAN, b);
    TEST_ASSERT_EQUAL_INT(2, m2.filledBins());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_still_constant_offset_10min);
    RUN_TEST(test_still_constant_offset_no_temperature);
    RUN_TEST(test_pilot_motion_with_warmup_10min);
    RUN_TEST(test_reference_without_estimation_drifts);
    RUN_TEST(test_boot_calibration_still);
    RUN_TEST(test_boot_calibration_timeout);
    RUN_TEST(test_boot_calibration_rejects_noise);
    RUN_TEST(test_boot_calibration_rejects_large_bias);
    RUN_TEST(test_step_clamp_limits_slow_pan);
    RUN_TEST(test_repeated_slow_pans_stay_in_envelope);
    RUN_TEST(test_accel_gate_blocks_slow_tilt);
    RUN_TEST(test_begin_and_restart_calibrated);
    RUN_TEST(test_still_calibrator);
    RUN_TEST(test_temperature_compensation_warmup);
    RUN_TEST(test_temperature_model);
    return UNITY_END();
}
