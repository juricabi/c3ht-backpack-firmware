#if defined(C3HT_EU_BUILD)
#include "eu_internal.h"
#include <WiFi.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_core_dump.h>
#include "common.h"
#include "config.h"
#include "c3ht_eu_pubkeys.h"

extern VrxBackpackConfig config;
extern unsigned long rebootTime;
void euConsoleInit();
void euConsoleLoop();

#define QUOTE_(x) #x
#define STR_(x) QUOTE_(x)
#if defined(C3HT_EU_RELEASE)
#define EU_REL "1"
#else
#define EU_REL "0"
#endif
// Read by sign_fw.py: the signer refuses images without this marker, with a
// different security version, or whose key list does not contain the key.
__attribute__((used)) static const char euBuildMarkerStr[] =
    "C3HTEU-BUILD;v=1;sv=" STR_(C3HT_EU_SECURITY_VERSION) ";rel=" EU_REL ";keys=" C3HT_EU_KEY_IDS_STR;
const char *const euBuildMarker = euBuildMarkerStr;

#if defined(C3HT_EU_RELEASE) && C3HT_EU_HAS_DEV_KEY
#error "C3HT_EU_RELEASE build with a DEV public key"
#endif

static const uint8_t LED_NOCRED[] = {8, 8, 8, 8, 8, 60};      // on/off pairs: 3 fast blinks, pause
static const uint8_t LED_PROMPT[] = {25, 25};                 // 2 Hz: "press BOOT to start Wi-Fi"

static bool radioUp = false;
static uint32_t radioUpMs = 0;
static bool validDone = false;
static uint32_t lastBootPress = 0;
static bool hadBootPress = false;
static uint32_t promptUntil = 0;
static bool remoteUsed = false;

static void eraseNet80211()
{
    nvs_handle_t h;
    if (nvs_open("nvs.net80211", NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void eraseCoreDump()
{
    size_t addr = 0, size = 0;
    if (esp_core_dump_image_get(&addr, &size) == ESP_OK) esp_core_dump_image_erase();
}

void euEarlySetup()
{
    WiFi.persistent(false);      // Wi-Fi driver config never stored in nvs.net80211
    eraseCoreDump();             // a previous crash dump may hold RAM secrets
    euConsoleInit();
    euStoreInit();
    euCredInit();
    if (euStoreOk() && euFirstEuBoot()) {
        // First boot of the EU firmware (normally right after USB flashing):
        // forget any stock-firmware Wi-Fi state and invalidate a stock image
        // that may remain in the other OTA slot.
        eraseNet80211();
        const esp_partition_t *other = esp_ota_get_next_update_partition(nullptr);
        if (other) esp_partition_erase_range(other, 0, 0x10000);
        euMarkInitialised();
    }
    euLogEvent(EV_BOOT, (uint16_t)esp_reset_reason(), false);
}

void euAfterConfigLoad()
{
    // Plaintext home Wi-Fi credentials left in EEPROM by stock firmware.
    if (config.GetSSID()[0] || config.GetPassword()[0]) {
        config.SetSSID("");
        config.SetPassword("");
        config.Commit();
    }
}

bool euWifiAllowed(const char *source)
{
    (void)source;
    if (euCredPresent()) return true;
    euSetLedNoCredential();
    euLogEvent(EV_NO_CREDENTIAL, 0, false);
    return false;
}

void euSetLedNoCredential()
{
    ledShowPattern(LED_NOCRED, sizeof(LED_NOCRED), 10000);
}

void euRemoteWifiRequest()
{
    if (remoteUsed) return;      // once per boot
    remoteUsed = true;
    euLogEvent(EV_REMOTE_WIFI_REQ, 0, false);
    if (!euCredPresent()) {
        euSetLedNoCredential();
        return;
    }
    promptUntil = millis() + 30000;
    if (promptUntil == 0) promptUntil = 1;
    ledShowPattern(LED_PROMPT, sizeof(LED_PROMPT), 30000);
}

bool euRemoteWifiPromptActive()
{
    return promptUntil && (int32_t)(millis() - promptUntil) < 0;
}

void euNoteBootPress()
{
    lastBootPress = millis();
    hadBootPress = true;
}

uint32_t euMsSinceBootPress()
{
    return hadBootPress ? millis() - lastBootPress : 0xFFFFFFFFu;
}

bool euBootHeld()
{
#if defined(PIN_BUTTON)
    return digitalRead(PIN_BUTTON) == LOW;
#else
    return false;
#endif
}

void euCredentialReset()
{
    // BOOT >= 10 s: back to the card password. Binding, IMU calibration and
    // orientation are kept (full reset: USB console or web UI).
    euClearUserPassword();
    euWebEndSessions();
    euLogEvent(EV_PW_RESET, 0, true);
    euLogFlush(true);
    rebootTime = millis() + 200;
}

void euFactoryResetFull()
{
    euWebEndSessions();
    euStoreFactoryWipe();
    config.SetDefaults();        // binding address, IMU calibration/orientation, stale STA fields
    eraseNet80211();
    eraseCoreDump();
    euLogEvent(EV_FACTORY_RESET, 0, true);
    euLogFlush(true);
    rebootTime = millis() + 200;
}

void euNoteRadioUp()
{
    if (!radioUp) {
        radioUp = true;
        radioUpMs = millis();
    }
}

void euLoop(uint32_t now)
{
    euConsoleLoop();
    euOtaLoop(now);
    // Rollback (verifyRollbackLater() returns true, src/c3ht_eu_hooks.cpp):
    // a new image is confirmed only after the radio came up and ran for 5 s.
    if (!validDone && radioUp && now - radioUpMs > 5000) {
        validDone = true;
        const esp_partition_t *run = esp_ota_get_running_partition();
        esp_ota_img_states_t s;
        if (run && esp_ota_get_state_partition(run, &s) == ESP_OK && s == ESP_OTA_IMG_PENDING_VERIFY) {
            esp_ota_mark_app_valid_cancel_rollback();
            euLogEvent(EV_UPDATE_CONFIRMED, C3HT_EU_SECURITY_VERSION, true);
        }
        euStoreMinSv(C3HT_EU_SECURITY_VERSION);   // only after the image is known good
    }
    if (promptUntil && !euRemoteWifiPromptActive()) promptUntil = 0;
    euLogFlush(false);
}
#endif
