// Compile-time unit checks for include/power_logic.h. No code is emitted; a
// failing check breaks every VRx build. (The repo has no native test env and
// build_src_filter skips *test* files, hence the name.)
#if defined(TARGET_VRX_BACKPACK)
#include "power_logic.h"

// --- boot counter ---
// Only power-on resets count.
static_assert(bootCountOnBoot(false, 0) == BOOTCOUNT_IGNORE, "brownout/WDT reset must not count");
static_assert(bootCountOnBoot(false, 2) == BOOTCOUNT_IGNORE, "non-power-on reset never triggers");
static_assert(bootCountOnBoot(false, 200) == BOOTCOUNT_IGNORE, "non-power-on reset never triggers");
// 1st and 2nd power cycles are counted later, the 3rd triggers.
static_assert(bootCountOnBoot(true, 0) == BOOTCOUNT_COUNT_LATER, "1st power cycle");
static_assert(bootCountOnBoot(true, 1) == BOOTCOUNT_COUNT_LATER, "2nd power cycle");
static_assert(bootCountOnBoot(true, 2) == BOOTCOUNT_TRIGGER, "3rd power cycle triggers");
// Upstream uint8_t arithmetic: a stored 255 wraps to 0 and does not trigger.
static_assert(bootCountOnBoot(true, 255) == BOOTCOUNT_COUNT_LATER, "255 wraps like upstream");
// Counted only after the settle time.
static_assert(!bootCountDue(true, 0), "not counted at boot");
static_assert(!bootCountDue(true, BOOTCOUNT_SETTLE_MS - 1), "not counted before 1.5 s");
static_assert(bootCountDue(true, BOOTCOUNT_SETTLE_MS), "counted at 1.5 s");
static_assert(!bootCountDue(false, 60000), "nothing pending, nothing counted");

#endif
