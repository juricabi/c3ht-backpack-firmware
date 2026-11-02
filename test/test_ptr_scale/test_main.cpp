// Host test for lib/HeadTrackerCore/PtrScale.h (head angle -> PTR channel).
// Run with: pio test -e native -f test_ptr_scale
#include <unity.h>
#include <math.h>

#include "PtrScale.h"

static const long OUT_MIN = 191;    // CRSF_CHANNEL_VALUE_1000
static const long OUT_MAX = 1792;   // CRSF_CHANNEL_VALUE_2000

// Arduino-ESP32 map(), the upstream conversion
static long arduinoMap(long x, long in_min, long in_max, long out_min, long out_max)
{
    const long run = in_max - in_min;
    const long rise = out_max - out_min;
    const long delta = x - in_min;
    return (delta * rise) / run + out_min;
}

void setUp() {}
void tearDown() {}

// 180 deg full scale must reproduce map(angle * 100, -18000, 18000, ...) exactly
// over the whole range getEuler() can return, including the negated yaw.
void test_180_is_identical_to_upstream()
{
    for (long i = -180000; i <= 180000; i++)
    {
        const float a = i / 1000.0f;
        const long ref = arduinoMap(a * 100, -180 * 100, 180 * 100, OUT_MIN, OUT_MAX);
        const long refNeg = arduinoMap(-a * 100, -180 * 100, 180 * 100, OUT_MIN, OUT_MAX);
        if (ref != ptrAngleToChannel(a, 180, OUT_MIN, OUT_MAX) || refNeg != ptrAngleToChannel(-a, 180, OUT_MIN, OUT_MAX))
        {
            TEST_FAIL_MESSAGE("mismatch against upstream map()");
        }
    }
}

void test_scaled_full_scale_and_centre()
{
    TEST_ASSERT_EQUAL_INT32(OUT_MAX, ptrAngleToChannel(90.0f, 90, OUT_MIN, OUT_MAX));
    TEST_ASSERT_EQUAL_INT32(OUT_MIN, ptrAngleToChannel(-90.0f, 90, OUT_MIN, OUT_MAX));
    TEST_ASSERT_EQUAL_INT32((OUT_MIN + OUT_MAX) / 2, ptrAngleToChannel(0.0f, 90, OUT_MIN, OUT_MAX));
    // 45 deg at 90 deg full scale = three quarters of the travel
    TEST_ASSERT_INT32_WITHIN(1, OUT_MIN + 3 * (OUT_MAX - OUT_MIN) / 4, ptrAngleToChannel(45.0f, 90, OUT_MIN, OUT_MAX));
}

// beyond the full-scale angle the output saturates; it never wraps (PREMORTEM D10)
void test_saturates_instead_of_wrapping()
{
    for (int a = 45; a <= 180; a++)
    {
        TEST_ASSERT_EQUAL_INT32(OUT_MAX, ptrAngleToChannel((float)a, 45, OUT_MIN, OUT_MAX));
        TEST_ASSERT_EQUAL_INT32(OUT_MIN, ptrAngleToChannel((float)-a, 45, OUT_MIN, OUT_MAX));
    }
    TEST_ASSERT_EQUAL_INT32(OUT_MAX, ptrAngleToChannel(1000.0f, 180, OUT_MIN, OUT_MAX));
    TEST_ASSERT_EQUAL_INT32(OUT_MIN, ptrAngleToChannel(-1000.0f, 180, OUT_MIN, OUT_MAX));
}

void test_monotonic()
{
    const long ranges[] = {1, 30, 45, 60, 90, 120, 180};
    for (unsigned r = 0; r < sizeof(ranges) / sizeof(ranges[0]); r++)
    {
        long prev = ptrAngleToChannel(-200.0f, ranges[r], OUT_MIN, OUT_MAX);
        for (int i = -2000; i <= 2000; i++)
        {
            const long v = ptrAngleToChannel(i / 10.0f, ranges[r], OUT_MIN, OUT_MAX);
            TEST_ASSERT_TRUE(v >= prev);
            TEST_ASSERT_TRUE(v >= OUT_MIN && v <= OUT_MAX);
            prev = v;
        }
    }
}

void test_nan_and_bad_range()
{
    TEST_ASSERT_EQUAL_INT32((OUT_MIN + OUT_MAX) / 2, ptrAngleToChannel(NAN, 90, OUT_MIN, OUT_MAX));
    // out-of-range full scale is clamped to 1..180
    TEST_ASSERT_EQUAL_INT32(ptrAngleToChannel(30.0f, 180, OUT_MIN, OUT_MAX), ptrAngleToChannel(30.0f, 500, OUT_MIN, OUT_MAX));
    TEST_ASSERT_EQUAL_INT32(ptrAngleToChannel(0.5f, 1, OUT_MIN, OUT_MAX), ptrAngleToChannel(0.5f, 0, OUT_MIN, OUT_MAX));
}

void test_compile_time_defaults_are_upstream()
{
    TEST_ASSERT_EQUAL_INT(180, HT_MAX_ANGLE_PAN);
    TEST_ASSERT_EQUAL_INT(180, HT_MAX_ANGLE_TILT);
    TEST_ASSERT_EQUAL_INT(180, HT_MAX_ANGLE_ROLL);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_180_is_identical_to_upstream);
    RUN_TEST(test_scaled_full_scale_and_centre);
    RUN_TEST(test_saturates_instead_of_wrapping);
    RUN_TEST(test_monotonic);
    RUN_TEST(test_nan_and_bad_range);
    RUN_TEST(test_compile_time_defaults_are_upstream);
    return UNITY_END();
}
