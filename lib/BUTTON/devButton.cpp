#include <Arduino.h>
#include "common.h"
#include "device.h"
#include "config.h"

#if defined(PIN_BUTTON)
#include "logging.h"
#include "button.h"

static Button<PIN_BUTTON, false> button;

extern unsigned long rebootTime;
void RebootIntoWifi(wifi_service_t service);

#if defined(C3HT_EU_BUILD)
// EU build (DESIGN.md section 2.8):
//  - short press: start Wi-Fi (or stop it when running), as before;
//  - hold ~1 s: arms the USB console 'show-card' / 'rekey' commands for 10 s;
//  - hold >= 10 s, then release: back to the card password (user password and
//    sessions cleared; binding and calibration kept). The short press does not
//    fire on that release, and a button already low at boot is ignored until released.
void euNoteBootPress();
void euCredentialReset();
void ledShowPattern(const uint8_t *seq, uint8_t count, uint32_t durationMs);
void ledCancelPattern();
static bool ignoreUntilRelease = false;
static bool resetArmed = false;
static bool countdownShown = false;
static uint8_t highSamples = 0;   // consecutive released (HIGH) samples
static const uint8_t LED_COUNTDOWN[] = {5, 5};
static const uint8_t LED_ARMED[] = {2, 2};

static void longPress()
{
    if (ignoreUntilRelease)
        return;
    uint8_t n = button.getLongCount();   // 0 at 0.5 s, +1 every 0.5 s
    if (n == 0)
        euNoteBootPress();
    else if (n == 3)
    {
        countdownShown = true;
        ledShowPattern(LED_COUNTDOWN, sizeof(LED_COUNTDOWN), 9000);
    }
    else if (n >= 19 && !resetArmed)
    {
        resetArmed = true;
        ledShowPattern(LED_ARMED, sizeof(LED_ARMED), 60000);
    }
}
#endif

static void shortPress()
{
#if defined(C3HT_EU_BUILD)
    if (ignoreUntilRelease)
        return;
#endif
    if (connectionState == wifiUpdate)
    {
        rebootTime = millis();
    }
    else
    {
        RebootIntoWifi(WIFI_SERVICE_UPDATE);
    }
}

static void initialize()
{
    button.OnShortPress = shortPress;
#if defined(C3HT_EU_BUILD)
    button.OnLongPress = longPress;
    ignoreUntilRelease = digitalRead(PIN_BUTTON) == LOW;
#endif
}

static int start()
{
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
#if defined(C3HT_EU_BUILD)
    int next = button.update();
    bool released = digitalRead(PIN_BUTTON) == HIGH;
    highSamples = released ? (highSamples < 255 ? highSamples + 1 : 255) : 0;
    // Button reports the release (short press) on the 2nd HIGH sample; clear the
    // boot-time ignore flag only after that, so that release is swallowed too.
    if (ignoreUntilRelease && highSamples >= 3)
        ignoreUntilRelease = false;
    if (resetArmed && released)
    {
        resetArmed = false;
        countdownShown = false;
        euCredentialReset();
    }
    else if (countdownShown && released)
    {
        // released before 10 s: no reset, stop the countdown pattern
        countdownShown = false;
        ledCancelPattern();
    }
    return next;
#else
    return button.update();
#endif
}

device_t Button_device = {
    .initialize = initialize,
    .start = start,
    .event = NULL,
    .timeout = timeout
};

#endif