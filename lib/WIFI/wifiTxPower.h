#pragma once

/*
 * ESP32 WiFi / ESP-NOW TX power, in the 0.25 dBm units of
 * esp_wifi_set_max_tx_power().
 *
 * Default: 19.5 dBm (upstream behaviour).
 * C3HT: -D C3HT_ESPNOW_TX_DBM=<dBm> (targets/c3ht.ini) caps both ESP-NOW and
 * the web-update AP/STA. The backpack link is shorter than 1 m to a few metres,
 * and 802.11b TX at 19.5 dBm peaks at ~335 mA, which the 3.3 V slot rail may
 * not deliver (C3HT DESIGN_RESEARCH H15, PREMORTEM D15).
 *
 * The C3HT EU build does not use this: it applies and reads back its own cap
 * (C3HT_EU_MAX_TX_QDBM, lib/C3HT_EU/src/eu_rf.cpp), and devWIFI.cpp is compiled
 * out there.
 */

#if defined(PLATFORM_ESP32)
#include <esp_wifi.h>

#if defined(C3HT_ESPNOW_TX_DBM)
  #define WIFI_TX_POWER_QDBM ((int8_t)((C3HT_ESPNOW_TX_DBM) * 4))
  static_assert(WIFI_TX_POWER_QDBM >= 8 && WIFI_TX_POWER_QDBM <= 80,
                "C3HT_ESPNOW_TX_DBM must be between 2 and 20 dBm");
#else
  #define WIFI_TX_POWER_QDBM ((int8_t)78) // WIFI_POWER_19_5dBm
#endif

// Only valid once esp_wifi_start() has returned (WiFi.mode(STA/AP) does that).
// Returns the read-back value (0.25 dBm units) or -1 on error.
static inline int8_t wifiApplyTxPower()
{
  if (esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM) != ESP_OK)
    return -1;
  int8_t p = -1;
  if (esp_wifi_get_max_tx_power(&p) != ESP_OK)
    return -1;
  return p;
}
#endif
