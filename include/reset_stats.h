#pragma once

/*
 * C3HT reset statistics (-D C3HT_BROWNOUT_STATS, ESP32 VRx only).
 *
 * resetStatsInit(), called first in setup():
 * - reads esp_reset_reason() once;
 * - updates a brownout counter kept in RTC_NOINIT memory: brownout resets
 *   since the last power-on (a power loss clears it);
 * - checks that the brownout detector is armed (enabled + reset) and
 *   re-arms it with the sdkconfig level if something disabled it.
 *
 * The values are logged and shown in the web UI (/config, EU /api/info).
 * Decision logic: include/power_logic.h (brownoutCountOnBoot()).
 */

#include <stdint.h>

#if defined(C3HT_BROWNOUT_STATS) && defined(PLATFORM_ESP32)
void resetStatsInit();
int resetStatsReason();             // esp_reset_reason_t of this boot
const char *resetStatsReasonName(); // e.g. "brownout"
uint16_t resetStatsBrownouts();     // brownout resets since power-on
bool resetStatsBrownoutArmed();     // detector enabled with reset
#endif
