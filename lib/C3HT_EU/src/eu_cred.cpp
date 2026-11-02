#if defined(C3HT_EU_BUILD)
#include "eu_internal.h"
#include <esp_efuse.h>
#include <esp_efuse_table.h>
#include <soc/efuse_reg.h>
#include <ctype.h>

/*
 * Per-device credential (DESIGN.md section 2).
 * Record (32 bytes): "C3H1" | ver=1 | flags | secret[16] | CRC16-CCITT-FALSE(0..21, big endian) | zero[8]
 * Lookup order: c3cred override slot (USB re-key) -> eFuse BLOCK_USR_DATA
 * (RS-error free) -> c3cred factory slot -> none. With no credential the AP
 * never starts.
 * The secret is re-read for each use and wiped afterwards; only the SSID and
 * the card fingerprint (both public) stay in RAM.
 */

static EuCredSource source = EU_CRED_NONE;
static uint8_t recFlags = 0;
static char ssid[16];
static char fp[5];
static bool rsError = false;

static bool recordValid(const uint8_t *r)
{
    if (memcmp(r, "C3H1", 4) != 0 || r[4] != 1) return false;
    uint16_t crc = (uint16_t)(r[22] << 8 | r[23]);
    return crc == euCrc16(r, 22);
}

static bool readRecord(EuCredSource src, uint8_t rec[EU_REC_LEN])
{
    bool ok = false;
    switch (src) {
    case EU_CRED_OVERRIDE:
        ok = euCredPartRead(EU_CRED_SLOT_OVERRIDE, rec, EU_REC_LEN) && recordValid(rec);
        break;
    case EU_CRED_EFUSE:
        if (REG_READ(EFUSE_RD_RS_ERR0_REG) & EFUSE_USR_DATA_FAIL) {
            rsError = true;
            break;
        }
        ok = esp_efuse_read_field_blob(ESP_EFUSE_USER_DATA, rec, EU_REC_LEN * 8) == ESP_OK && recordValid(rec);
        break;
    case EU_CRED_FLASH:
        ok = euCredPartRead(EU_CRED_SLOT_FACTORY, rec, EU_REC_LEN) && recordValid(rec);
        break;
    default:
        break;
    }
    if (!ok) euWipe(rec, EU_REC_LEN);
    return ok;
}

static bool loadSecret(uint8_t secret[16])
{
    uint8_t rec[EU_REC_LEN];
    if (source == EU_CRED_NONE || !readRecord(source, rec)) return false;
    memcpy(secret, rec + 6, 16);
    euWipe(rec, sizeof(rec));
    return true;
}

void euCredInit()
{
    static const EuCredSource order[] = {EU_CRED_OVERRIDE, EU_CRED_EFUSE, EU_CRED_FLASH};
    uint8_t rec[EU_REC_LEN];
    source = EU_CRED_NONE;
    for (EuCredSource s : order) {
        if (readRecord(s, rec)) {
            source = s;
            recFlags = rec[5];
            uint8_t k[3];
            euHkdf(rec + 6, 16, "ssid", k, 3);
            snprintf(ssid, sizeof(ssid), "C3HT-%02X%02X%02X", k[0], k[1], k[2]);
            euHkdf(rec + 6, 16, "fp", k, 3);
            euCrockford(k, 3, 4, fp);
            break;
        }
    }
    euWipe(rec, sizeof(rec));
    if (source == EU_CRED_NONE) {
        ssid[0] = 0;
        strcpy(fp, "----");
    }
}

EuCredSource euCredSource() { return source; }
bool euCredPresent() { return source != EU_CRED_NONE; }
bool euCredIsDev() { return recFlags & 1; }
const char *euSsid() { return ssid; }
const char *euFingerprint() { return fp; }
bool euEfuseRsError() { return rsError; }

const char *euCredSourceName()
{
    switch (source) {
    case EU_CRED_OVERRIDE: return "override";
    case EU_CRED_EFUSE: return "efuse";
    case EU_CRED_FLASH: return "flash";
    default: return "none";
    }
}

bool euCardPassword(char out[21])
{
    uint8_t secret[16], k[13];
    if (!loadSecret(secret)) {
        out[0] = 0;
        return false;
    }
    euHkdf(secret, 16, "wifi-psk", k, 13);
    euCrockford(k, 13, 20, out);
    euWipe(secret, sizeof(secret));
    euWipe(k, sizeof(k));
    return true;
}

bool euUserPasswordSet()
{
    char buf[65];
    bool set = euGetUserPassword(buf, sizeof(buf));
    euWipe(buf, sizeof(buf));
    return set;
}

bool euActivePassphrase(char *out, size_t len)
{
    if (source == EU_CRED_NONE || len < 64) return false;
    if (euGetUserPassword(out, len)) return true;
    char card[21];
    bool ok = euCardPassword(card);
    if (ok) strlcpy(out, card, len);
    euWipe(card, sizeof(card));
    return ok;
}

// Card input is normalised on the web login only (Wi-Fi WPA2/WPA3 cannot be):
// upper case, '-' and spaces removed, O->0, I/L->1 (Crockford).
static void normaliseCard(const char *in, char *out, size_t len)
{
    size_t n = 0;
    for (const char *p = in; *p && n + 1 < len; p++) {
        char c = (char)toupper((unsigned char)*p);
        if (c == '-' || c == ' ') continue;
        if (c == 'O') c = '0';
        if (c == 'I' || c == 'L') c = '1';
        out[n++] = c;
    }
    out[n] = 0;
}

bool euCheckLogin(const char *input)
{
    if (source == EU_CRED_NONE || !input) return false;
    char ref[65];
    char in[65];
    uint8_t h1[32], h2[32];
    bool ok;
    if (euGetUserPassword(ref, sizeof(ref))) {
        // A user password replaces the card password completely (no fallback).
        strlcpy(in, input, sizeof(in));
    } else {
        if (!euCardPassword(ref)) return false;
        normaliseCard(input, in, sizeof(in));
    }
    // compare fixed-length digests in constant time
    euSha256((const uint8_t *)ref, strlen(ref), h1);
    euSha256((const uint8_t *)in, strlen(in), h2);
    ok = euCtEqual(h1, h2, 32) && strlen(input) <= 64;
    euWipe(ref, sizeof(ref));
    euWipe(in, sizeof(in));
    euWipe(h1, sizeof(h1));
    euWipe(h2, sizeof(h2));
    return ok;
}

// ---------------------------------------------------------------- policy
static const char *const BLOCKLIST[] = {
    "expresslrs", "password", "passwort", "qwerty", "qwertz", "azerty", "123456", "654321", "abcdef",
    "iloveyou", "letmein", "welcome", "admin", "c3ht", "backpack", "monkey", "dragon", "football",
    "baseball", "sunshine", "princess", "master", "shadow", "superman", "batman", "trustno1",
    "fpv", "drone", "orqa", "skyzone", "hdzero", "goggles", "summer", "winter", "spring", "autumn",
    "000000", "111111", "aaaaaa", "asdfgh", "zxcvbn", "1q2w3e", "changeme", "default", "secret"};

static int estimateBits(const char *pw)
{
    // Very conservative: remove blocklisted words, collapse runs, then
    // log2(pool) per remaining char.
    char low[65];
    size_t n = strlen(pw);
    for (size_t i = 0; i <= n; i++) low[i] = (char)tolower((unsigned char)pw[i]);
    for (const char *w : BLOCKLIST) {
        char *p;
        size_t wl = strlen(w);
        while ((p = strstr(low, w)) != nullptr) memmove(p, p + wl, strlen(p + wl) + 1);
    }
    bool lo = false, up = false, di = false, sy = false;
    for (const char *p = pw; *p; p++) {
        if (islower((unsigned char)*p)) lo = true;
        else if (isupper((unsigned char)*p)) up = true;
        else if (isdigit((unsigned char)*p)) di = true;
        else sy = true;
    }
    int pool = (lo ? 26 : 0) + (up ? 26 : 0) + (di ? 10 : 0) + (sy ? 33 : 0);
    int bitsPerChar10 = pool >= 95 ? 66 : pool >= 62 ? 60 : pool >= 36 ? 52 : pool >= 26 ? 47 : pool >= 10 ? 33 : 10;
    size_t eff = 0;
    for (size_t i = 0; low[i]; i++) {
        if (i > 0 && low[i] == low[i - 1]) continue;                   // runs
        if (i > 1 && low[i] - low[i - 1] == low[i - 1] - low[i - 2] &&  // sequences
            abs(low[i] - low[i - 1]) == 1) continue;
        eff++;
    }
    euWipe(low, sizeof(low));
    return (int)(eff * bitsPerChar10 / 10);
}

const char *euPasswordPolicy(const char *pw)
{
    size_t n = strlen(pw);
    if (n < 16) return "at least 16 characters";
    if (n > 63) return "at most 63 characters";
    for (const char *p = pw; *p; p++)
        if (*p < 0x20 || *p > 0x7e) return "printable ASCII only";
    if (strcasecmp(pw, ssid) == 0) return "must not be the network name";
    // whole password made of one repeated chunk ("abcdabcdabcd...")
    for (size_t k = 1; k <= n / 2; k++) {
        if (n % k) continue;
        bool rep = true;
        for (size_t i = k; i < n && rep; i++) rep = pw[i] == pw[i % k];
        if (rep) return "repeating pattern";
    }
    char card[21];
    if (euCardPassword(card)) {
        bool same = strcasecmp(card, pw) == 0;
        euWipe(card, sizeof(card));
        if (same) return "must differ from the card password";
    }
    if (estimateBits(pw) < 60) return "too predictable (needs about 60 bits; use 'Generate')";
    return nullptr;
}

const char *euSetUserPassword(const char *pw)
{
    const char *err = euPasswordPolicy(pw);
    if (err) return err;
    if (!euSetUserPasswordRaw(pw)) return "storage error";
    return nullptr;
}

bool euGeneratePassword(char out[21])
{
    uint8_t r[13];
    euRandom(r, sizeof(r));
    euCrockford(r, sizeof(r), 20, out);
    euWipe(r, sizeof(r));
    return true;
}

bool euRekey()
{
    if (!euCredPartPresent()) return false;
    uint8_t rec[EU_REC_LEN] = {0};
    memcpy(rec, "C3H1", 4);
    rec[4] = 1;
    rec[5] = recFlags;
    euRandom(rec + 6, 16);
    uint16_t crc = euCrc16(rec, 22);
    rec[22] = crc >> 8;
    rec[23] = crc & 0xff;
    bool ok = euCredPartWrite(EU_CRED_SLOT_OVERRIDE, rec, EU_REC_LEN);
    euWipe(rec, sizeof(rec));
    if (ok) {
        euClearUserPassword();
        euCredInit();
    }
    return ok;
}
#endif
