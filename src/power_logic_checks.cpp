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

// --- WiFi idle timeout (5 min) ---
static_assert(!wifiIdleExpired(0, 0, 300000), "not expired at start");
static_assert(!wifiIdleExpired(1000, 300999, 300000), "1 ms before 5 min");
static_assert(wifiIdleExpired(1000, 301000, 300000), "expired at 5 min");
static_assert(!wifiIdleExpired(0xFFFFF000u, 0x00001000u, 300000), "millis() wrap: 8 s idle");
static_assert(wifiIdleExpired(0xFFFFF000u, 300000u, 300000), "millis() wrap: > 5 min idle");
static_assert(!wifiIdleExpired(0, 0xFFFFFFFFu, 0), "timeout 0 disables");
static_assert(!wifiIdleExpired(5001, 5000, 300000), "activity newer than now (other task) is not idle");

// --- brownout counter (RTC memory) ---
static_assert(resetStatsValid(RESETSTATS_MAGIC, 3, (uint16_t)~3), "valid record");
static_assert(!resetStatsValid(RESETSTATS_MAGIC, 3, 3), "count/check mismatch is invalid");
static_assert(!resetStatsValid(0, 0, 0xFFFF), "wrong magic is invalid");
static_assert(!resetStatsValid(0, 0, 0), "zeroed RAM is invalid");
// Power-on starts from 0 whatever RTC RAM holds.
static_assert(brownoutCountOnBoot(true, true, false, 7) == 0, "power-on clears");
static_assert(brownoutCountOnBoot(false, true, false, 7) == 0, "power-on clears (garbage)");
// Brownout resets count up from a valid record, or restart at 1.
static_assert(brownoutCountOnBoot(true, false, true, 0) == 1, "1st brownout");
static_assert(brownoutCountOnBoot(true, false, true, 4) == 5, "next brownout");
static_assert(brownoutCountOnBoot(false, false, true, 4) == 1, "brownout, record lost");
static_assert(brownoutCountOnBoot(true, false, true, 0xFFFF) == 0xFFFF, "saturates");
// Other resets (watchdog, panic, software) keep the count.
static_assert(brownoutCountOnBoot(true, false, false, 4) == 4, "WDT/SW reset keeps count");
static_assert(brownoutCountOnBoot(false, false, false, 4) == 0, "WDT/SW reset, record lost");

#endif
