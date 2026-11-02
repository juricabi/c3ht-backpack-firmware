#pragma once

/*
 * Pure decision logic for the C3HT power / boot behaviour (no Arduino or IDF
 * calls), so it can be checked at compile time (src/power_logic_checks.cpp)
 * and on a host. C++11 constexpr.
 */

#include <stdint.h>

/////////// Boot counter (3 power cycles -> binding / WiFi) ///////////
// C3HT_BOOTCOUNT_POWERON_ONLY: only a power-on reset counts, and only once the
// board has stayed up for BOOTCOUNT_SETTLE_MS, so brownout / watchdog / panic
// resets and supply bounces never open WiFi or binding (PREMORTEM D15).

#define BOOTCOUNT_TRIGGER_COUNT 3     // the 3rd counted boot triggers
#define BOOTCOUNT_SETTLE_MS     1500  // uptime before a power-on is counted

enum bootCountAction_e : uint8_t
{
    BOOTCOUNT_IGNORE,      // not a power-on reset: leave the stored count alone
    BOOTCOUNT_TRIGGER,     // enough counted power cycles: binding / WiFi now
    BOOTCOUNT_COUNT_LATER, // store count + 1 once BOOTCOUNT_SETTLE_MS is reached
};

// storedCount = boots already counted (config.GetBootCount()).
// Same threshold arithmetic as upstream checkIfInBindingMode() (uint8_t).
constexpr bootCountAction_e bootCountOnBoot(bool powerOnReset, uint8_t storedCount)
{
    return !powerOnReset ? BOOTCOUNT_IGNORE
         : ((uint8_t)(storedCount + 1) >= BOOTCOUNT_TRIGGER_COUNT ? BOOTCOUNT_TRIGGER
                                                                  : BOOTCOUNT_COUNT_LATER);
}

constexpr bool bootCountDue(bool pending, uint32_t nowMs)
{
    return pending && nowMs >= BOOTCOUNT_SETTLE_MS;
}

/////////// WiFi update mode idle timeout ///////////
// C3HT_WIFI_IDLE_TIMEOUT_S: reboot to normal mode after this long without a
// client. millis() wrap-safe (unsigned difference). lastActivityMs is written
// by the web server task and may be a little newer than nowMs: a "negative"
// age (bit 31 set) means "just active", never "expired".

constexpr bool wifiIdleExpired(uint32_t lastActivityMs, uint32_t nowMs, uint32_t timeoutMs)
{
    return timeoutMs != 0
        && (uint32_t)(nowMs - lastActivityMs) >= timeoutMs
        && (uint32_t)(nowMs - lastActivityMs) < 0x80000000u;
}
