#pragma once

// Run-time gyroscope bias estimation for the head tracker.
//
// Two stages, both working on gyro samples that already have the stored
// (web UI) calibration applied:
//  1. Boot sample: average a window of samples while the board is still
//     (per-axis peak-to-peak below a threshold). Gives a usable bias within
//     a couple of seconds of power-on. Abandoned after a timeout if the board
//     never stays still long enough; stage 2 then does the work.
//  2. FusionOffset (x-io Fusion): a slow high-pass on the gyro that adapts
//     whenever the gyro has read below 3 deg/s for 5 s, which tracks the
//     temperature drift of the zero-rate offset during a session.
// The total estimate is clamped per axis. Nothing is written to flash.
//
// Plain C++ with no Arduino dependency, so it can be unit tested on the host
// (test/test_gyro_bias).

#include "Fusion.h"     // wraps the Fusion C headers in extern "C"

struct GyroBiasSettings
{
    unsigned int sampleRateHz;        // IMU output data rate
    unsigned int bootSamples;         // samples averaged for the boot bias (0 = no boot sample)
    unsigned int bootTimeoutSamples;  // give up the boot sample after this many samples
    float bootStillP2pDps;            // max per-axis peak-to-peak inside the boot window
    float maxBiasDps;                 // clamp for the bias estimate, per axis
};

// Defaults: 100 Hz, 2 s still window, 15 s timeout, 1 deg/s p-p, 5 deg/s clamp.
GyroBiasSettings GyroBiasDefaultSettings(unsigned int sampleRateHz);

class GyroBiasEstimator
{
public:
    GyroBiasEstimator() { begin(GyroBiasDefaultSettings(100)); }
    void begin(const GyroBiasSettings &settings);

    // Feed one gyro sample (deg/s) and get the bias-corrected sample back.
    FusionVector update(FusionVector gyro);

    FusionVector bias() const { return m_offset.gyroscopeOffset; }
    bool bootSampleDone() const { return m_bootDone; }
    bool bootSampleAccepted() const { return m_bootAccepted; }

private:
    void bootSample(const FusionVector &gyro);
    void restartWindow(const FusionVector &gyro);
    void clampBias();

    GyroBiasSettings m_settings;
    FusionOffset m_offset;

    bool m_bootDone;
    bool m_bootAccepted;
    unsigned int m_bootSeen;
    unsigned int m_windowCount;
    float m_sum[3];
    float m_min[3];
    float m_max[3];
};
