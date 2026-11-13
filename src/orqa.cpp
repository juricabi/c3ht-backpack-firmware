#include "orqa.h"
#include "logging.h"

GENERIC_CRC8 ghst_crc(GHST_CRC_POLY);

void Orqa::SendIndexCmd(uint8_t index)
{
    uint8_t band = GetBand(index);
    uint8_t channel = GetChannel(index);
    uint8_t currentGHSTChannel = GHSTChannel(band, channel);
    uint16_t currentFrequency = GetFrequency(index);
    SendGHSTUpdate(currentFrequency, currentGHSTChannel);
}


// ---- DVR record on/off over GHST (frame type 0x23) ----
// The recording command already reaches us via Vrx_main.cpp
// (MSP_ELRS_BACKPACK_SET_RECORDING_STATE). We emit the matching GHST frame to
// the goggle, which drives its DVR to this absolute state. delay is in seconds
// (same as skyzone_msp.cpp); the send is deferred in Loop().
void Orqa::SetRecordingState(uint8_t recordingState, uint16_t delay)
{
    DBGLN("Orqa SetRecordingState = %d delay = %d", recordingState, delay);
    m_recordingState = recordingState ? 1 : 0;
    m_delay = (uint32_t)delay * 1000; // seconds -> ms
    m_delayStartMillis = millis();
    if (m_delay == 0)
        SendRecordingState();
}

void Orqa::Loop(uint32_t now)
{
    // Orqa::Loop() hides ModuleBase::Loop(), which holds the on-board head
    // tracker sender (HAS_HEADTRACKING), so chain to it.
    ModuleBase::Loop(now);

    // millis(), not `now`: SetRecordingState() runs from the ESP-NOW receive
    // callback and can start the delay after loop() sampled `now`; the unsigned
    // difference would then wrap and the delayed command would go out at once.
    if (m_delay != 0 && (millis() - m_delayStartMillis >= m_delay))
    {
        SendRecordingState();
        m_delay = 0;
    }
}

void Orqa::SendRecordingState()
{
    // 14-byte GHST frame, type 0x23 = DVR, payload[0] = desired state
    // (1 = record, 0 = stop). Same CRC-8 (poly 0xD5 over [2..12]) as the VTX
    // frame, so the goggle's CRC check accepts it.
    uint8_t packet[] = {
        0x82, 0x0C, 0x23,                 // Header: addr, length, type 0x23
        m_recordingState,                 // payload[0] = desired DVR state
        0x00, 0x00, 0x00, 0x00,           // payload (unused)
        0x00, 0x00, 0x00, 0x00, 0x00,     // payload (unused)
        0x00 };                           // CRC
    uint8_t crc = ghst_crc.calc(&packet[2], 11);
    packet[13] = crc;

    for (uint8_t i = 0; i < 14; i++)
    {
        Serial.write(packet[i]);
    }
}

void Orqa::SendGHSTUpdate(uint16_t freq, uint8_t ghstChannel)
{
    uint8_t packet[] = {
        0x82, 0x0C, 0x20,                                       // Header
        0x00, (uint8_t)(freq & 0xFF), (uint8_t)(freq >> 8),     // Frequency
        0x01, 0x00, ghstChannel,                                // Band & Channel
        0x00, 0x00, 0x00, 0x00,                                 // Power Level?
        0x00 };                                                 // CRC
    uint8_t crc = ghst_crc.calc(&packet[2], 11);
    packet[13] = crc;

    for(uint8_t i = 0; i < 14; i++)
    {
        Serial.write(packet[i]);
    }
}

uint8_t Orqa::GHSTChannel(uint8_t band, uint8_t channel)
{
    // ELRS Bands:  A, B, E, I/F, R, L
    // Orqa/Rapidfire Bands: I/F, R, E, B, A, L, X
    uint8_t ghstChannel = 0x00;

    switch(band)
    {
    case 0x01:
        ghstChannel |= 0x50;
        break;
    case 0x02:
        ghstChannel |= 0x40;
        break;
    case 0x03:
        ghstChannel |= 0x30;
        break;
    case 0x04:
        ghstChannel |= 0x10;
        break;
    case 0x05:
        ghstChannel |= 0x20;
        break;
    case 0x06:
        ghstChannel |= 0x60;
        break;
    }
    ghstChannel |= channel;

    return ghstChannel;
}