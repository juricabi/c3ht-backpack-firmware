#pragma once

#include "module_base.h"
#include <channels.h>
#include <crc.h>

#define VRX_UART_BAUD   100000
#define VRX_BOOT_DELAY  7000
#define GHST_CRC_POLY   0xD5

class Orqa : public ModuleBase
{
public:
    void SendIndexCmd(uint8_t index);
    void SetRecordingState(uint8_t recordingState, uint16_t delay);
    void Loop(uint32_t now);
private:
    uint8_t GHSTChannel(uint8_t band, uint8_t channel);
    void SendGHSTUpdate(uint16_t freq, uint8_t ghstChannel);
    void SendRecordingState();
    uint8_t  m_recordingState = 0;
    uint32_t m_delay = 0;            // ms remaining; 0 = nothing pending
    uint32_t m_delayStartMillis = 0;
};