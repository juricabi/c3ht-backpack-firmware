#if defined(C3HT_EU_BUILD)
#include "eu_internal.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_partition.h>

/*
 * Persistent EU state.
 *  - 'fctry' NVS partition, namespace "c3ht_eu": upw (user password), min_sv,
 *    boots, init, evlog. The partition is listed AFTER 'nvs' in
 *    targets/c3ht_eu_partitions.csv, so Arduino's initArduino() (which formats
 *    the first NVS partition on NO_FREE_PAGES/NEW_VERSION) never touches it.
 *  - 'c3cred' raw data partition (subtype 0x40): credential records and the
 *    optional user public key, written with esptool in production or by the
 *    USB console. Never formatted by firmware.
 * Writes are rare and coalesced (event log at most once per 60 s unless an
 * important event), so an authenticated client cannot wear out the flash.
 */

#define FCTRY_PART "fctry"
#define NS "c3ht_eu"
#define CRED_SUBTYPE ((esp_partition_subtype_t)0x40)
#define LOG_N 32

static bool storeOk = false;
static nvs_handle_t nh = 0;
static const esp_partition_t *credPart = nullptr;
static uint32_t bootCount = 0;
static bool firstBoot = false;

struct LogBlob {
    uint8_t head;     // next write index
    uint8_t n;        // valid entries
    uint8_t ver;
    uint8_t pad;
    EuLogEntry e[LOG_N];
};
static LogBlob evlog;
static bool logDirty = false;
static bool importantPending = false;
static uint32_t logLastFlush = 0;

bool euStoreInit()
{
    credPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, CRED_SUBTYPE, "c3cred");
    esp_err_t err = nvs_flash_init_partition(FCTRY_PART);
    bool wasReset = false;
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Only this partition, never 'nvs'. Loses user password/events/min_sv;
        // the effective minimum security version stays >= the running image's.
        nvs_flash_erase_partition(FCTRY_PART);
        err = nvs_flash_init_partition(FCTRY_PART);
        wasReset = true;
    }
    if (err != ESP_OK) {
        storeOk = false;
        return false;
    }
    if (nvs_open_from_partition(FCTRY_PART, NS, NVS_READWRITE, &nh) != ESP_OK) {
        storeOk = false;
        return false;
    }
    storeOk = true;
    nvs_get_u32(nh, "boots", &bootCount);
    bootCount++;
    nvs_set_u32(nh, "boots", bootCount);
    uint8_t init = 0;
    firstBoot = nvs_get_u8(nh, "init", &init) != ESP_OK || init != 1;
    size_t sz = sizeof(evlog);
    if (nvs_get_blob(nh, "evlog", &evlog, &sz) != ESP_OK || sz != sizeof(evlog) || evlog.ver != 1 ||
        evlog.head >= LOG_N || evlog.n > LOG_N) {
        memset(&evlog, 0, sizeof(evlog));
        evlog.ver = 1;
    }
    nvs_commit(nh);
    if (wasReset) euLogEvent(EV_STORE_RESET, 0, true);
    return true;
}

bool euStoreOk() { return storeOk; }
bool euCredPartPresent() { return credPart != nullptr; }
uint32_t euBootCount() { return bootCount; }
bool euFirstEuBoot() { return firstBoot; }

void euMarkInitialised()
{
    if (!storeOk) return;
    nvs_set_u8(nh, "init", 1);
    nvs_commit(nh);
    firstBoot = false;
}

bool euCredPartRead(uint32_t off, uint8_t *buf, size_t len)
{
    if (!credPart || off + len > credPart->size) return false;
    return esp_partition_read(credPart, off, buf, len) == ESP_OK;
}

bool euCredPartWrite(uint32_t off, const uint8_t *buf, size_t len)
{
    if (!credPart || off + len > 4096 || credPart->size < 4096) return false;
    uint8_t *sector = (uint8_t *)malloc(4096);
    if (!sector) return false;
    bool ok = esp_partition_read(credPart, 0, sector, 4096) == ESP_OK;
    if (ok) {
        memcpy(sector + off, buf, len);
        ok = esp_partition_erase_range(credPart, 0, 4096) == ESP_OK &&
             esp_partition_write(credPart, 0, sector, 4096) == ESP_OK;
    }
    if (ok) {
        uint8_t *chk = (uint8_t *)malloc(len);
        ok = chk && esp_partition_read(credPart, off, chk, len) == ESP_OK && memcmp(chk, buf, len) == 0;
        free(chk);
    }
    euWipe(sector, 4096);
    free(sector);
    return ok;
}

uint32_t euStoredMinSv()
{
    uint32_t v = 0;
    if (storeOk) nvs_get_u32(nh, "min_sv", &v);
    return v;
}

void euStoreMinSv(uint32_t sv)
{
    if (!storeOk) return;
    if (sv > euStoredMinSv()) {
        nvs_set_u32(nh, "min_sv", sv);
        nvs_commit(nh);
    }
}

bool euGetUserPassword(char *buf, size_t len)
{
    if (!storeOk) return false;
    size_t l = len;
    if (nvs_get_str(nh, "upw", buf, &l) != ESP_OK) {
        if (len) buf[0] = 0;
        return false;
    }
    return buf[0] != 0;
}

bool euSetUserPasswordRaw(const char *pw)
{
    if (!storeOk) return false;
    return nvs_set_str(nh, "upw", pw) == ESP_OK && nvs_commit(nh) == ESP_OK;
}

void euClearUserPassword()
{
    if (!storeOk) return;
    nvs_erase_key(nh, "upw");
    nvs_commit(nh);
}

void euStoreFactoryWipe()
{
    if (!storeOk) return;
    nvs_erase_key(nh, "upw");
    memset(&evlog, 0, sizeof(evlog));
    evlog.ver = 1;
    nvs_erase_key(nh, "evlog");
    nvs_commit(nh);
    logDirty = false;
}

// ---------------------------------------------------------------- event log
void euLogEvent(EuEvent ev, uint16_t detail, bool important)
{
    uint32_t up = millis() / 1000;
    uint8_t last = (evlog.head + LOG_N - 1) % LOG_N;
    // Coalesce floods of the same event in the same boot into one entry.
    if (evlog.n > 0 && evlog.e[last].type == ev && evlog.e[last].boot == bootCount && evlog.e[last].detail == detail &&
        evlog.e[last].count < 255) {
        evlog.e[last].count++;
        evlog.e[last].uptime_s = up;
    } else {
        EuLogEntry &e = evlog.e[evlog.head];
        e.boot = bootCount;
        e.uptime_s = up;
        e.type = ev;
        e.count = 1;
        e.detail = detail;
        evlog.head = (evlog.head + 1) % LOG_N;
        if (evlog.n < LOG_N) evlog.n++;
    }
    logDirty = true;
    if (important) importantPending = true;
    euLogFlush(false);
}

void euLogFlush(bool force)
{
    // force: only before a reboot. Otherwise at most one write per 2 s while an
    // important event is pending, and one per 60 s for the rest.
    if (!storeOk || !logDirty) return;
    uint32_t now = millis();
    uint32_t interval = importantPending ? 2000 : 60000;
    if (!force && logLastFlush != 0 && now - logLastFlush < interval) return;
    nvs_set_blob(nh, "evlog", &evlog, sizeof(evlog));
    nvs_commit(nh);
    logDirty = false;
    importantPending = false;
    logLastFlush = now;
}

int euLogGet(EuLogEntry *out, int max)
{
    int n = 0;
    for (int i = 0; i < evlog.n && n < max; i++) {
        int idx = (evlog.head + LOG_N - 1 - i) % LOG_N;
        out[n++] = evlog.e[idx];
    }
    return n;
}

const char *euEventName(uint8_t t)
{
    static const char *const names[] = {
        "none", "boot", "ap-start", "ap-stop", "login-ok", "login-fail", "lockout",
        "password-changed", "password-reset", "update-accepted", "update-rejected", "update-confirmed",
        "factory-reset", "rekey", "key-added", "key-removed", "remote-wifi-request",
        "no-credential", "store-reset", "txpower-fail", "ws-auth-fail"};
    return t < sizeof(names) / sizeof(names[0]) ? names[t] : "?";
}
#endif
