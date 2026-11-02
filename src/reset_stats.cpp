// C3HT reset statistics: brownout counter in RTC memory + brownout detector
// check. See include/reset_stats.h. Empty translation unit unless
// -D C3HT_BROWNOUT_STATS (targets/c3ht.ini [c3ht_common]).
#if defined(C3HT_BROWNOUT_STATS) && defined(PLATFORM_ESP32)
#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <sdkconfig.h>
#include <soc/soc_caps.h>
#include <soc/rtc_cntl_reg.h>
#include <brownout.h> // esp_brownout_init() (esp_system/port/public_compat)

#include "logging.h"
#include "power_logic.h"
#include "reset_stats.h"

// The prebuilt Arduino-ESP32 C3 sdkconfig enables the detector at level 7,
// the lowest threshold (~2.5 V; the level voltages are estimates). Keep it:
// a higher level trips on a sagging 3.3 V slot rail that the chip would ride
// through (C3HT DESIGN_RESEARCH H16). Levels 2-3 (~3 V and above) are refused.
#if defined(CONFIG_ESP32C3_BROWNOUT_DET)
  #define C3HT_BOD_LVL CONFIG_ESP32C3_BROWNOUT_DET_LVL
#elif defined(CONFIG_ESP_BROWNOUT_DET)
  #define C3HT_BOD_LVL CONFIG_ESP_BROWNOUT_DET_LVL
#else
  #error "C3HT: the framework sdkconfig has the brownout detector disabled"
#endif
static_assert(C3HT_BOD_LVL >= 4 && C3HT_BOD_LVL <= 7,
              "C3HT: brownout level must be 4..7 (~2.9-2.5 V) for a 3.3 V rail");

// On chips with a brownout reset (C3), esp_brownout_init() only calls
// brownout_hal_config() {level, enabled, reset_enabled, flash/RF power-down}
// and registers no ISR (checked in the prebuilt libesp_system.a of
// framework-arduinoespressif32 3.20016), so calling it again is idempotent.
#if !SOC_BROWNOUT_RESET_SUPPORTED
  #error "C3HT_BROWNOUT_STATS expects a chip with a brownout reset (ESP32-C3)"
#endif
#define BOD_ARMED_BITS (RTC_CNTL_BROWN_OUT_ENA | RTC_CNTL_BROWN_OUT_RST_ENA)

static RTC_NOINIT_ATTR struct
{
    uint32_t magic;
    uint16_t brownouts;
    uint16_t brownoutsInv;
} rtcStats;

static int bootReason = ESP_RST_UNKNOWN;
static bool bodArmed = false;

static bool readBodArmed()
{
    return (REG_READ(RTC_CNTL_BROWN_OUT_REG) & BOD_ARMED_BITS) == BOD_ARMED_BITS;
}

void resetStatsInit()
{
    esp_reset_reason_t reason = esp_reset_reason();
    bootReason = (int)reason;

    bool valid = resetStatsValid(rtcStats.magic, rtcStats.brownouts, rtcStats.brownoutsInv);
    uint16_t count = brownoutCountOnBoot(valid, reason == ESP_RST_POWERON,
                                         reason == ESP_RST_BROWNOUT, rtcStats.brownouts);
    rtcStats.brownouts = count;
    rtcStats.brownoutsInv = (uint16_t)~count;
    rtcStats.magic = RESETSTATS_MAGIC;

    // IDF startup arms the detector (esp_brownout_init()); nothing in this
    // firmware disables it. Re-arm with the sdkconfig level if it is off.
    bodArmed = readBodArmed();
    if (!bodArmed)
    {
        esp_brownout_init();
        bodArmed = readBodArmed();
        DBGLN("brownout detector was off, re-armed: %d", bodArmed);
    }

    DBGLN("reset reason = %s (%d), brownouts since power-on = %u, BOD level %d %s",
          resetStatsReasonName(), bootReason, count, C3HT_BOD_LVL, bodArmed ? "armed" : "OFF");
}

int resetStatsReason() { return bootReason; }
uint16_t resetStatsBrownouts() { return rtcStats.brownouts; }
bool resetStatsBrownoutArmed() { return bodArmed; }

const char *resetStatsReasonName()
{
    switch (bootReason)
    {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "unknown";
    }
}
#endif
