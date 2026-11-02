#include <Arduino.h>
#include "options.h"

#include <ArduinoJson.h>
#include <StreamString.h>
#include "EspFlashStream.h"
#if defined(PLATFORM_ESP8266)
#include <FS.h>
#else
#include <SPIFFS.h>
#endif
#if defined(PLATFORM_ESP32)
#include <esp_partition.h>
#include "esp_ota_ops.h"
#endif

#define QUOTE(arg) #arg
#define STR(macro) QUOTE(macro)
const unsigned char target_name[] = "\xBE\xEF\xCA\xFE" STR(TARGET_NAME);
const uint8_t target_name_size = sizeof(target_name);

const char PROGMEM compile_options[] = {
#ifdef MY_BINDING_PHRASE
    "-DMY_BINDING_PHRASE=\"" STR(MY_BINDING_PHRASE) "\" "
#endif
};

firmware_options_t firmwareOptions;

// Discriminator value used to determine if the device has been reflashed and therefore
// the SPIFSS settings are obsolete and the flashed settings should be used in preference
uint32_t flash_discriminator;

static StreamString builtinOptions;
String& getOptions()
{
    return builtinOptions;
}

void saveOptions(Stream &stream, bool customised)
{
    JsonDocument doc;

    if (firmwareOptions.hasUID)
    {
        JsonArray uid = doc.createNestedArray("uid");
        copyArray(firmwareOptions.uid, sizeof(firmwareOptions.uid), uid);
    }
    if (firmwareOptions.home_wifi_ssid[0])
    {
        doc["wifi-ssid"] = firmwareOptions.home_wifi_ssid;
        doc["wifi-password"] = firmwareOptions.home_wifi_password;
    }
    doc["product-name"] = firmwareOptions.product_name;
    doc["flash-discriminator"] = flash_discriminator;

    serializeJson(doc, stream);
}

void saveOptions()
{
    File options = SPIFFS.open("/options.json", "w");
    saveOptions(options, true);
    options.close();
}

/**
 * @brief:  Checks if the strmFlash currently is pointing to something that looks like
 *          a string (not all 0xFF). Position in the stream will not be changed.
 * @return: true if appears to have a string
 */
bool options_HasStringInFlash(EspFlashStream &strmFlash)
{
    uint32_t firstBytes;
    size_t pos = strmFlash.getPosition();
    strmFlash.readBytes((uint8_t *)&firstBytes, sizeof(firstBytes));
    strmFlash.setPosition(pos);

    return firstBytes != 0xffffffff;
}

#if !defined(C3HT_EU_BUILD)
/**
 * @brief:  Internal read options from either the flash stream at the end of the sketch or the options.json file
 *          Fills the firmwareOptions variable
 * @return: true if either was able to be parsed
 */
static void options_LoadFromFlashOrFile(EspFlashStream &strmFlash)
{
    JsonDocument flashDoc;
    JsonDocument spiffsDoc;
    bool hasFlash = false;
    bool hasSpiffs = false;

    strmFlash.setPosition(0);
    if (options_HasStringInFlash(strmFlash))
    {
        DeserializationError error = deserializeJson(flashDoc, strmFlash);
        if (error)
        {
            return;
        }
        hasFlash = true;
    }

    // load options.json from the SPIFFS partition
    File file = SPIFFS.open("/options.json", "r");
    if (file && !file.isDirectory())
    {
        DeserializationError error = deserializeJson(spiffsDoc, file);
        if (!error)
        {
            hasSpiffs = true;
        }
    }

    JsonDocument &doc = flashDoc;
    if (hasFlash && hasSpiffs)
    {
        if (flashDoc["flash-discriminator"] == spiffsDoc["flash-discriminator"])
        {
            doc = spiffsDoc;
        }
    }
    else if (hasSpiffs)
    {
        doc = spiffsDoc;
    }

    if (doc["uid"].is<JsonArray>())
    {
        copyArray(doc["uid"], firmwareOptions.uid, sizeof(firmwareOptions.uid));
        firmwareOptions.hasUID = true;
    }
    else
    {
        firmwareOptions.hasUID = false;
    }
    strlcpy(firmwareOptions.home_wifi_ssid, doc["wifi-ssid"] | "", sizeof(firmwareOptions.home_wifi_ssid));
    strlcpy(firmwareOptions.home_wifi_password, doc["wifi-password"] | "", sizeof(firmwareOptions.home_wifi_password));
    strlcpy(firmwareOptions.product_name, doc["product-name"] | "", sizeof(firmwareOptions.product_name));
    flash_discriminator = doc["flash-discriminator"] | 0U;

    builtinOptions.clear();
    saveOptions(builtinOptions, doc["customised"] | false);
}
#endif // !C3HT_EU_BUILD

#if defined(C3HT_EU_BUILD)
/*
 * EU build: the options block after the image is NOT covered by the update
 * signature (the ELRS Configurator re-patches it), so it is untrusted input.
 * Only uid (6 bytes), product-name (printable, bounded) and
 * flash-discriminator are used. Home Wi-Fi keys are ignored and SPIFFS
 * /options.json is never read (the EU partition table has no SPIFFS).
 */
bool options_init()
{
    memset(&firmwareOptions, 0, sizeof(firmwareOptions));
    flash_discriminator = 0;
    const esp_partition_t *runningPart = esp_ota_get_running_partition();
    EspFlashStream strmFlash;
    strmFlash.setBaseAddress((runningPart ? runningPart->address : 0) + ESP.getSketchSize());
    strmFlash.setPosition(0);
    JsonDocument doc;
    if (options_HasStringInFlash(strmFlash))
    {
        if (deserializeJson(doc, strmFlash, DeserializationOption::NestingLimit(2)))
            doc.clear();
    }
    JsonArray uid = doc["uid"].as<JsonArray>();
    if (!uid.isNull() && uid.size() == 6)
    {
        bool ok = true;
        for (size_t i = 0; i < 6; i++)
        {
            int v = uid[i] | -1;
            if (v < 0 || v > 255)
                ok = false;
            else
                firmwareOptions.uid[i] = (uint8_t)v;
        }
        firmwareOptions.hasUID = ok;
        if (!ok)
            memset(firmwareOptions.uid, 0, sizeof(firmwareOptions.uid));
    }
    const char *pn = doc["product-name"] | "";
    size_t n = 0;
    for (; pn[n] && n < sizeof(firmwareOptions.product_name) - 1; n++)
        firmwareOptions.product_name[n] = (pn[n] >= 0x20 && pn[n] < 0x7f && pn[n] != '"' && pn[n] != '<' && pn[n] != '>') ? pn[n] : '_';
    firmwareOptions.product_name[n] = 0;
    flash_discriminator = doc["flash-discriminator"] | 0U;
    builtinOptions.clear();
    saveOptions(builtinOptions, false);
    return true;
}
#else
bool options_init()
{
    uint32_t baseAddr = 0;
#if defined(PLATFORM_ESP32)
    SPIFFS.begin(true);
    const esp_partition_t *runningPart = esp_ota_get_running_partition();
    if (runningPart)
    {
        baseAddr = runningPart->address;
    }
#else
    SPIFFS.begin();
    // ESP8266 sketch baseAddr is always 0
#endif

    EspFlashStream strmFlash;
    strmFlash.setBaseAddress(baseAddr + ESP.getSketchSize());

    // options.json
    options_LoadFromFlashOrFile(strmFlash);

    // hardware.json
    // bool hasHardware = hardware_init(strmFlash);

    return true; //hasHardware;
}
#endif
