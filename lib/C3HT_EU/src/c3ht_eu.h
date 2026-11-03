#pragma once
/*
 * C3HT EU build (compile-time variant, -D C3HT_EU_BUILD).
 * Design and rationale: firmware/eu-build/DESIGN.md in the C3HT project.
 *
 * This library is compiled for every VRx env (PlatformIO LDF ignores #ifdef),
 * but every translation unit is empty unless C3HT_EU_BUILD is defined, so the
 * stock C3HT_* firmware is unchanged.
 */
#if defined(C3HT_EU_BUILD)

#include <Arduino.h>

#if !defined(C3HT_EU_SECURITY_VERSION)
#error "C3HT_EU_SECURITY_VERSION must be defined for EU builds"
#endif
#if defined(HOME_WIFI_SSID) || defined(HOME_WIFI_PASSWORD) || defined(MY_AP_PASSWORD) || defined(AUTO_WIFI_ON_INTERVAL)
#error "EU build: home Wi-Fi / static AP password / auto Wi-Fi defines are not allowed"
#endif
#if defined(DEBUG_ELRS_WIFI)
#error "EU build: DEBUG_ELRS_WIFI (Wi-Fi at every boot) is not allowed"
#endif
#if defined(C3HT_EU_RELEASE) && (defined(DEBUG_LOG) || defined(DEBUG_LOG_VERBOSE))
#error "EU release build: DEBUG_LOG is not allowed"
#endif

// RF: esp_wifi_set_max_tx_power() unit is 0.25 dBm (8 = 2 dBm, the API minimum). Default 78 = 19.5 dBm, the
// upstream ExpressLRS value, on every board (owner decision 2026-11-03). Applied before the first frame and read
// back (radio stopped above the cap). EN 300 328 (mean e.i.r.p. <= 20 dBm) is confirmed per board by the lab; if a
// board measures higher, lower it in its env with -D C3HT_EU_MAX_TX_QDBM=<n>.
#ifndef C3HT_EU_MAX_TX_QDBM
#define C3HT_EU_MAX_TX_QDBM 78
#endif
#if (C3HT_EU_MAX_TX_QDBM < 8) || (C3HT_EU_MAX_TX_QDBM > 78)
#error "C3HT_EU_MAX_TX_QDBM must be 8..78 (2..19.5 dBm)"
#endif
// IMU fitted on every C3HT board: TDK ICM-42607-C, WHO_AM_I 0x61 (the EU selftest requires it).
#ifndef C3HT_EU_IMU_WHOAMI
#define C3HT_EU_IMU_WHOAMI 0x61
#endif

// ---- lifecycle hooks (src/Vrx_main.cpp) ---------------------------------
void euEarlySetup();                 // very first call in setup()
void euAfterConfigLoad();            // after config.Load()
void euLoop(uint32_t now);           // every loop() iteration (also in Wi-Fi mode)
void euNoteRadioUp();                // ESP-NOW or AP came up successfully
bool euEspNowRadioSetup();           // replaces tx-power/protocol/fake-begin in SetSoftMACAddress

// ---- Wi-Fi triggers -------------------------------------------------------
// Returns true if the AP may start (a device credential exists). Otherwise
// shows the "no credential" LED pattern and returns false.
bool euWifiAllowed(const char *source);
// Unauthenticated remote request (ESP-NOW MSP_ELRS_SET_VRX_BACKPACK_WIFI_MODE):
// only shows a prompt; the user confirms with a BOOT press within 30 s.
void euRemoteWifiRequest();
bool euRemoteWifiPromptActive();

// ---- button (lib/BUTTON/devButton.cpp) ----------------------------------
void euNoteBootPress();              // any BOOT press (gates USB show-card / rekey)
void euCredentialReset();            // BOOT held >= 10 s, then released

// ---- credential -----------------------------------------------------------
bool euCredPresent();

// ---- LED (lib/LED/devLED.cpp) --------------------------------------------
void ledShowPattern(const uint8_t *seq, uint8_t count, uint32_t durationMs);
void ledCancelPattern();

#endif // C3HT_EU_BUILD
