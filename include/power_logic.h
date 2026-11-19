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

/////////// Brownout reset counter (RTC memory) ///////////
// C3HT_BROWNOUT_STATS: brownout resets since the last power-on, kept in
// RTC_NOINIT memory. That survives brownout, watchdog, panic and software
// resets but not a power loss. The record is {magic, count, ~count}; anything
// else (power-on garbage, a supply drop deep enough to lose RTC RAM) is
// invalid and restarts the count.

#define RESETSTATS_MAGIC 0xC3B0B0C3u

constexpr bool resetStatsValid(uint32_t magic, uint16_t count, uint16_t countInv)
{
    return magic == RESETSTATS_MAGIC && (uint16_t)~count == countInv;
}

// Brownout count after this boot. A power-on reset (or an invalid record)
// starts again from 0; a brownout reset adds one, saturating at 0xFFFF.
constexpr uint16_t brownoutBase(bool recordValid, bool powerOnReset, uint16_t stored)
{
    return (powerOnReset || !recordValid) ? 0 : stored;
}

constexpr uint16_t brownoutCountOnBoot(bool recordValid, bool powerOnReset, bool brownoutReset, uint16_t stored)
{
    return (brownoutReset && brownoutBase(recordValid, powerOnReset, stored) != 0xFFFF)
         ? (uint16_t)(brownoutBase(recordValid, powerOnReset, stored) + 1)
         : brownoutBase(recordValid, powerOnReset, stored);
}

/////////// Never-bound VRx backpack ///////////
// A VRx backpack that was never bound, by the radio or by a phrase set on its web page, has an all-zero group
// address (fresh config) or an all-0xFF one (erased flash). Such a board starts binding mode by itself at a
// real power-on.

constexpr bool groupAddressByteUnset(uint8_t b0, uint8_t b)
{
    return (b0 == 0x00 || b0 == 0xFF) && b == b0;
}

constexpr bool groupAddressUnset(const uint8_t a[6])
{
    return groupAddressByteUnset(a[0], a[0]) && groupAddressByteUnset(a[0], a[1]) &&
           groupAddressByteUnset(a[0], a[2]) && groupAddressByteUnset(a[0], a[3]) &&
           groupAddressByteUnset(a[0], a[4]) && groupAddressByteUnset(a[0], a[5]);
}
