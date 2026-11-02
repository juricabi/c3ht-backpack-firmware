#if defined(C3HT_EU_BUILD)
/*
 * USB-Serial-JTAG console (HWCDC, IO18/IO19). This is the physical-access
 * service/installation path. It is NEVER parsed on Serial (UART0 = goggle link).
 * Output is written only while a USB host has the port open, never from the
 * head-tracking path. Commands:
 *   help | status | selftest | events | reboot
 *   show-card                 (within 10 s of a BOOT press)
 *   rekey                     (within 10 s of a BOOT press) new secret in the override slot
 *   addkey <130 hex> | delkey (while BOOT is held) user OTA public key
 *   factory-reset <last 4 characters of the SSID>
 */
#include "eu_internal.h"
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include "common.h"
#include "options.h"
#include "devHeadTracker.h"

extern unsigned long rebootTime;
extern const char *VERSION;

#define CONSOLE_LINE_MAX 160
static char line[CONSOLE_LINE_MAX];
static size_t ll = 0;
static bool overflow = false;

void euConsoleInit()
{
    USBSerial.setTxTimeoutMs(0);
    USBSerial.begin();
}

static void out(const char *fmt, ...)
{
    if (!HWCDC::isConnected()) return;
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    USBSerial.print(buf);
}

static void printCard()
{
    char pw[65];
    bool user = euUserPasswordSet();
    if (!euActivePassphrase(pw, sizeof(pw))) {
        out("no credential\r\n");
        return;
    }
    out("SSID      %s\r\n", euSsid());
    out("password  %s%s\r\n", pw, user ? "   (user-set password; the printed card is not active)" : "");
    out("QR        WIFI:T:WPA;S:%s;P:%s;;\r\n", euSsid(), pw);
    out("card ID   %s\r\n", euFingerprint());
    euWipe(pw, sizeof(pw));
}

static void cmdStatus()
{
    char keys[160];
    euOtaKeyIds(keys, sizeof(keys));
    const esp_partition_t *run = esp_ota_get_running_partition();
    out("C3HT EU  target %s  version %s\r\n", &((const char *)target_name)[4], VERSION);
    out("build    %s\r\n", euBuildMarker);
    out("security version %u  minimum accepted %u\r\n", (unsigned)C3HT_EU_SECURITY_VERSION,
        (unsigned)euEffectiveMinSv());
    out("credential %s%s  SSID %s  card ID %s  user password %s\r\n", euCredSourceName(),
        euCredIsDev() ? " (DEV)" : "", euSsid()[0] ? euSsid() : "-", euFingerprint(),
        euUserPasswordSet() ? "set" : "no (card password active)");
    if (euEfuseRsError()) out("WARNING: eFuse BLOCK_USR_DATA has RS errors (ignored)\r\n");
    out("OTA keys %s  (%s)\r\n", keys[0] ? keys : "none", euOtaEnabled() ? "OTA enabled" : "OTA disabled, USB only");
    out("partitions fctry %s  c3cred %s  running %s\r\n", euStoreOk() ? "ok" : "MISSING",
        euCredPartPresent() ? "ok" : "MISSING", run ? run->label : "?");
    if (!euStoreOk() || !euCredPartPresent())
        out("WARNING: EU partition table not installed - flash all four images over USB\r\n");
    out("boot %u  uptime %us  state %d  AP %s  tx power %d (0.25 dBm units)\r\n", (unsigned)euBootCount(),
        (unsigned)(millis() / 1000), (int)connectionState, euApActive() ? "on" : "off", (int)euReadTxPower());
}

static void cmdSelftest()
{
    bool ok = true;
    bool cred = euCredPresent();
    out("credential   %s\r\n", cred ? euCredSourceName() : "FAIL (none)");
    ok &= cred;
    bool parts = euStoreOk() && euCredPartPresent();
    out("partitions   %s\r\n", parts ? "ok" : "FAIL");
    ok &= parts;
    bool imu = getHeadTrackerState() != STATE_ERROR;
    out("imu          %s\r\n", imu ? "ok" : "FAIL (not detected)");
    ok &= imu;
    int8_t p = euReadTxPower();
    bool pw = p > 0 && p <= C3HT_EU_MAX_TX_QDBM;
    out("tx power     %d -> %s\r\n", (int)p, pw ? "ok (<= 2 dBm)" : "FAIL");
    ok &= pw;
    bool ota = esp_ota_get_next_update_partition(nullptr) != nullptr;
    out("ota slot     %s  keys %d\r\n", ota ? "ok" : "FAIL", euOtaKeyCount());
    ok &= ota;
    out("SELFTEST %s fp=%s dev=%d\r\n", ok ? "PASS" : "FAIL", euFingerprint(), euCredIsDev() ? 1 : 0);
}

static void cmdEvents()
{
    EuLogEntry e[32];
    int n = euLogGet(e, 32);
    for (int i = 0; i < n; i++)
        out("boot %u +%us  %s x%u (%u)\r\n", (unsigned)e[i].boot, (unsigned)e[i].uptime_s, euEventName(e[i].type),
            e[i].count, e[i].detail);
    if (!n) out("no events\r\n");
}

static void handle(char *cmd)
{
    char *arg = strchr(cmd, ' ');
    if (arg) *arg++ = 0;
    else arg = cmd + strlen(cmd);
    if (!strcmp(cmd, "help")) {
        out("help status selftest events reboot show-card rekey addkey <hex> delkey factory-reset <ssid-last4>\r\n");
    } else if (!strcmp(cmd, "status")) {
        cmdStatus();
    } else if (!strcmp(cmd, "selftest")) {
        cmdSelftest();
    } else if (!strcmp(cmd, "events")) {
        cmdEvents();
    } else if (!strcmp(cmd, "reboot")) {
        rebootTime = millis() + 100;
    } else if (!strcmp(cmd, "show-card")) {
        if (euMsSinceBootPress() > 10000) out("press BOOT, then run show-card within 10 s\r\n");
        else printCard();
    } else if (!strcmp(cmd, "rekey")) {
        if (euMsSinceBootPress() > 10000) out("press BOOT, then run rekey within 10 s\r\n");
        else if (!euRekey()) out("rekey failed (c3cred partition missing?)\r\n");
        else {
            euLogEvent(EV_REKEY, 0, true);
            out("new credential (the old card no longer works):\r\n");
            printCard();
        }
    } else if (!strcmp(cmd, "addkey")) {
        if (!euBootHeld()) out("hold BOOT while sending addkey\r\n");
        else {
            const char *e = euUserKeyAdd(arg);
            out("%s\r\n", e ? e : "user OTA key stored");
        }
    } else if (!strcmp(cmd, "delkey")) {
        if (!euBootHeld()) out("hold BOOT while sending delkey\r\n");
        else out("%s\r\n", euUserKeyRemove() ? "user OTA key removed" : "failed");
    } else if (!strcmp(cmd, "factory-reset")) {
        const char *s = euSsid();
        size_t n = strlen(s);
        if (n < 4 || strcasecmp(arg, s + n - 4) != 0) out("type: factory-reset <last 4 characters of the SSID>\r\n");
        else {
            out("factory reset, rebooting\r\n");
            euFactoryResetFull();
        }
    } else if (cmd[0]) {
        out("unknown command (help)\r\n");
    }
}

void euConsoleLoop()
{
    int budget = 64;
    while (budget-- > 0 && USBSerial.available()) {
        int c = USBSerial.read();
        if (c < 0) break;
        if (c == '\r' || c == '\n') {
            if (!overflow && ll) {
                line[ll] = 0;
                handle(line);
            }
            euWipe(line, sizeof(line));
            ll = 0;
            overflow = false;
        } else if (ll < CONSOLE_LINE_MAX - 1) {
            line[ll++] = (char)c;
        } else {
            overflow = true;
        }
    }
}
#endif
