#if defined(C3HT_EU_BUILD)
#include "eu_internal.h"
#include <esp_wifi.h>

/*
 * RF limits (DESIGN.md section 5). 2.0 dBm conducted (API minimum, value 8 in
 * 0.25 dBm units) for SoftAP and ESP-NOW, applied right after esp_wifi_start
 * and after every mode change, then read back. If the read-back is not <= the
 * cap, the radio is stopped (fail closed). 802.11 LR is not enabled in the EU
 * build. BLE: see src/c3ht_eu_hooks.cpp (controller memory released at boot).
 */

bool euCapTxPower()
{
    if (esp_wifi_set_max_tx_power(C3HT_EU_MAX_TX_QDBM) != ESP_OK) return false;
    int8_t p = 0;
    if (esp_wifi_get_max_tx_power(&p) != ESP_OK) return false;
    return p > 0 && p <= C3HT_EU_MAX_TX_QDBM;
}

static bool rfStopped = false;   // set when the radio was stopped (fail closed)

// True only while the RF subsystem runs: esp_fill_random() is a true RNG then.
bool euRfRunning()
{
    return !rfStopped && euReadTxPower() > 0;
}

int8_t euReadTxPower()
{
    int8_t p = -1;
    if (esp_wifi_get_max_tx_power(&p) != ESP_OK) return -1;
    return p;
}

// Called by SetSoftMACAddress() after WiFi.mode(WIFI_STA) (STA started, not
// connected, nothing transmitted yet). Replaces WiFi.setTxPower(19.5 dBm),
// the LR protocol bit and the fake WiFi.begin() that sent probe requests.
bool euEspNowRadioSetup()
{
    if (!euCapTxPower()) {
        euLogEvent(EV_TXPOWER_FAIL, 1, true);
        rfStopped = true;
        esp_wifi_stop();
        return false;
    }
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    if (!euCapTxPower()) {
        euLogEvent(EV_TXPOWER_FAIL, 2, true);
        rfStopped = true;
        esp_wifi_stop();
        return false;
    }
    return true;
}
#endif
