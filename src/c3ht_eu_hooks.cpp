// C3HT EU build: strong overrides of weak Arduino-ESP32 hooks.
// Empty translation unit for every other build.
#if defined(C3HT_EU_BUILD)
#include <Arduino.h>

// Rollback: do not let initArduino() confirm a freshly updated image before
// setup() ran. lib/C3HT_EU/src/eu_main.cpp confirms it once the radio has been
// up for 5 s (esp_ota_mark_app_valid_cancel_rollback).
extern "C" bool verifyRollbackLater() { return true; }

// BLE is never used: returning false makes initArduino() release the BT
// controller memory (esp_bt_controller_mem_release), so BLE cannot be started.
extern "C" bool btInUse() { return false; }
#endif
