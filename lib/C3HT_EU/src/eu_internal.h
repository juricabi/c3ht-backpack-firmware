#pragma once
// Internal interfaces between the C3HT EU modules. Not used outside lib/C3HT_EU.
#if defined(C3HT_EU_BUILD)
#include <Arduino.h>
#include "c3ht_eu.h"

// ---- crypto helpers (eu_crypto.cpp) --------------------------------------
void euSha256(const uint8_t *data, size_t len, uint8_t out[32]);
void euHmacSha256(const uint8_t *key, size_t klen, const uint8_t *msg, size_t mlen, uint8_t out[32]);
// HKDF-SHA256 with the fixed salt "C3HT-EU-v1"; outlen <= 32.
void euHkdf(const uint8_t *ikm, size_t ikmlen, const char *info, uint8_t *out, size_t outlen);
// First 5*nchars bits of data, Crockford Base32, NUL-terminated (out: nchars+1).
void euCrockford(const uint8_t *data, size_t datalen, int nchars, char *out);
bool euCtEqual(const void *a, const void *b, size_t len);
uint16_t euCrc16(const uint8_t *data, size_t len);
void euRandom(void *buf, size_t len);
void euWipe(void *buf, size_t len);
void euHex(const uint8_t *in, size_t len, char *out);        // out: 2*len+1
bool euUnhex(const char *in, uint8_t *out, size_t len);      // exactly 2*len hex chars

// ---- persistent store (eu_store.cpp) --------------------------------------
// 'fctry' NVS partition (listed after 'nvs', so Arduino never formats it) and
// the raw 4 KiB 'c3cred' partition (credential records, user public key).
#define EU_CRED_SLOT_FACTORY  0x000
#define EU_CRED_SLOT_OVERRIDE 0x100
#define EU_CRED_SLOT_USERKEY  0x200
#define EU_REC_LEN 32

bool euStoreInit();
bool euStoreOk();          // fctry NVS usable
bool euCredPartPresent();
bool euCredPartRead(uint32_t off, uint8_t *buf, size_t len);
bool euCredPartWrite(uint32_t off, const uint8_t *buf, size_t len);   // read-modify-erase-write
uint32_t euStoredMinSv();
void euStoreMinSv(uint32_t sv);
bool euGetUserPassword(char *buf, size_t len);
bool euSetUserPasswordRaw(const char *pw);
void euClearUserPassword();
uint32_t euBootCount();
bool euFirstEuBoot();
void euMarkInitialised();
void euStoreFactoryWipe();  // user password + events (keeps min_sv, boot count)

enum EuEvent : uint8_t {
    EV_NONE = 0, EV_BOOT, EV_AP_START, EV_AP_STOP, EV_LOGIN_OK, EV_LOGIN_FAIL, EV_LOCKOUT,
    EV_PW_CHANGE, EV_PW_RESET, EV_UPDATE_OK, EV_UPDATE_REJECT, EV_UPDATE_CONFIRMED,
    EV_FACTORY_RESET, EV_REKEY, EV_KEY_ADDED, EV_KEY_REMOVED, EV_REMOTE_WIFI_REQ,
    EV_NO_CREDENTIAL, EV_STORE_RESET, EV_TXPOWER_FAIL, EV_WS_AUTH_FAIL
};
void euLogEvent(EuEvent ev, uint16_t detail, bool important);
void euLogFlush(bool force);
struct EuLogEntry { uint32_t boot; uint32_t uptime_s; uint8_t type; uint8_t count; uint16_t detail; };
int euLogGet(EuLogEntry *out, int max);   // newest first
const char *euEventName(uint8_t type);

// ---- credential (eu_cred.cpp) ---------------------------------------------
enum EuCredSource : uint8_t { EU_CRED_NONE = 0, EU_CRED_OVERRIDE, EU_CRED_EFUSE, EU_CRED_FLASH };
void euCredInit();
EuCredSource euCredSource();
const char *euCredSourceName();
bool euCredIsDev();
const char *euSsid();
const char *euFingerprint();
bool euUserPasswordSet();
// Passphrase the SoftAP uses (user password if set, else the card password).
bool euActivePassphrase(char *out, size_t len);
bool euCardPassword(char out[21]);
bool euCheckLogin(const char *input);
// Returns nullptr on success or a short reason string.
const char *euPasswordPolicy(const char *pw);
const char *euSetUserPassword(const char *pw);
bool euGeneratePassword(char out[21]);
bool euRekey();
bool euEfuseRsError();

// ---- signed OTA (eu_ota.cpp) ------------------------------------------------
enum EuOtaState : uint8_t { OTA_IDLE = 0, OTA_RECEIVING, OTA_VERIFYING, OTA_DONE, OTA_FAILED };
bool euOtaEnabled();             // at least one public key
int  euOtaKeyCount();
void euOtaKeyIds(char *out, size_t len);
const char *euOtaBegin(size_t total, bool allowOtherEuTarget);   // nullptr = ok
const char *euOtaWrite(const uint8_t *data, size_t len, size_t index);
void euOtaFinish();
void euOtaAbort(const char *why);
void euOtaLoop(uint32_t now);
EuOtaState euOtaState();
const char *euOtaMessage();
uint32_t euOtaProgress();        // bytes received
uint32_t euOtaTotal();
uint32_t euEffectiveMinSv();
bool euUserKeyPresent();
const char *euUserKeyAdd(const char *hex130);
bool euUserKeyRemove();

// ---- AP / web (eu_web.cpp) ----------------------------------------------------
bool euApActive();
uint32_t euApRemainingS();
void euWebEndSessions();

// ---- radio (eu_rf.cpp) ------------------------------------------------------------
bool euCapTxPower();             // apply and read back C3HT_EU_MAX_TX_QDBM
int8_t euReadTxPower();          // -1 if Wi-Fi not started
bool euRfRunning();              // RF on: hardware RNG is a TRNG

// ---- misc (eu_main.cpp) ---------------------------------------------------------
uint32_t euMsSinceBootPress();
bool euBootHeld();
void euFactoryResetFull();       // everything except eFuse, c3cred and min_sv; reboots
void euSetLedNoCredential();
extern const char *const euBuildMarker;

#endif
