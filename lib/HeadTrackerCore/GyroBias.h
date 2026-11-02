#pragma once

// Run-time gyroscope bias estimation for the head tracker
// (IMU_SELECTION.md F1/F2, PREMORTEM D5, DESIGN_RESEARCH H17).
//
// Works on gyro samples (deg/s) that already have the stored web-UI
// calibration applied, in the sensor frame, together with the accel (g)
// and, when the IMU has one, the die temperature (degC).
//
//  F1 boot calibration (StillCalibrator): drop the first 200 ms, then wait
//     up to 10 s for a 1 s window in which every gyro axis has a standard
//     deviation < 0.15 deg/s, |accel| stays within 1 +- 0.03 g and the
//     gravity direction moves < 0.5 deg. Then the bias is the plain mean of
//     the next 2 s (same gates). A mean above 3 deg/s on any axis is
//     rejected (stored calibration kept). The web "ci" command runs the same
//     routine.
//  F2 runtime estimate: a stillness detector like Fusion's FusionOffset
//     (corrected gyro below a threshold on every axis for 3 s) plus the
//     same accel gate, then a 0.02 Hz low-pass of the gyro into the bias.
//     Once an estimate has been accepted the threshold is 1 deg/s and each
//     stillness episode may move the bias by at most
//     0.05 deg/s + 0.02 deg/s/K * |temperature change since the last update|,
//     because a gyro cannot tell a slow steady head turn from bias. Before
//     that (boot calibration failed) it runs like FusionOffset: 3 deg/s
//     threshold and no step clamp, until 25 s of still time have converged it.
//  The total estimate is clamped per axis. Nothing is written to flash.
//
// Plain C++ with no Arduino dependency, so it can be unit tested on the host
// (test/test_gyro_bias).

#include "Fusion.h"     // wraps the Fusion C headers in extern "C"

struct GyroBiasSettings
{
    unsigned int sampleRateHz;      // IMU output data rate

    // F1 stillness-gated mean
    float bootDiscardS;             // samples dropped after IMU start
    float bootGateS;                // stillness window before averaging
    float bootMeanS;                // averaging window (0 = no boot calibration)
    float bootTimeoutS;             // give up after this long (after the discard)
    float stillStdDps;              // max per-axis standard deviation in a window
    float maxBootBiasDps;           // reject a mean above this on any axis

    // stillness accel gate (F1 and F2)
    float accelNormTolG;            // | |a| - 1 g | limit
    float accelDirTolDeg;           // gravity direction change limit

    // F2 runtime estimate
    float thresholdDps;             // stillness threshold once an estimate is accepted
    float thresholdUnlockedDps;     // stillness threshold before that
    float timeoutS;                 // still time before the estimate adapts
    float cutoffHz;                 // low-pass cut-off of the adaptation
    float unlockStillS;             // still (adapting) time that counts as converged
    float stepClampDps;             // max bias change per stillness episode ...
    float stepClampDpsPerK;         // ... plus this per K since the last update
    float maxBiasDps;               // clamp for the total estimate, per axis
};

// 100 Hz defaults as listed above.
GyroBiasSettings GyroBiasDefaultSettings(unsigned int sampleRateHz);

// Stillness-gated mean of the gyro (F1). Used for the boot calibration and
// for the web-UI calibration.
class StillCalibrator
{
public:
    enum Result
    {
        RUNNING,
        ACCEPTED,       // mean() is the bias
        REJECTED,       // still, but the mean exceeds maxBootBiasDps
        TIMED_OUT       // never still long enough
    };

    StillCalibrator() { begin(GyroBiasDefaultSettings(100)); }
    void begin(const GyroBiasSettings &settings);
    Result update(const FusionVector &gyro, const FusionVector &accel);
    Result result() const { return m_result; }
    FusionVector mean() const { return m_mean; }

private:
    enum Phase { DISCARD, GATE, MEAN, DONE };

    void restartWindow(Phase phase, const FusionVector &accel);
    bool accelStill(const FusionVector &accel) const;

    unsigned int m_discardN, m_gateN, m_meanN, m_timeoutN;
    float m_stillStd, m_maxBias, m_normTol, m_cosDirTol;

    Phase m_phase;
    Result m_result;
    unsigned int m_seen;
    unsigned int m_n;
    float m_wMean[3];
    float m_wM2[3];
    FusionVector m_ref;
    FusionVector m_mean;
};

class GyroBiasEstimator
{
public:
    GyroBiasEstimator() { begin(GyroBiasDefaultSettings(100)); }
    void begin(const GyroBiasSettings &settings);

    // A new stored calibration was just measured at this temperature: the
    // residual bias is zero, no boot calibration needed.
    void restartCalibrated(float temperatureC);

    // Feed one sample (gyro deg/s, accel g, die temperature degC or NAN) and
    // get the bias-corrected gyro back.
    FusionVector update(FusionVector gyro, const FusionVector &accel, float temperatureC);

    FusionVector bias() const { return m_bias; }
    bool bootSampleDone() const { return m_bootDone; }
    bool bootSampleAccepted() const { return m_bootAccepted; }
    bool locked() const { return m_locked; }
    bool adapting() const { return m_active; }
    float temperature() const { return m_temp; }

private:
    void acceptEstimate();
    void clampBias(FusionVector &b) const;
    bool accelStill(const FusionVector &accel) const;
    void updateTemperature(float temperatureC);
    FusionVector modelBias() const;

    GyroBiasSettings m_settings;
    StillCalibrator m_boot;
    float m_k;                  // low-pass coefficient per sample
    unsigned int m_timeoutN;
    unsigned int m_unlockN;
    float m_cosDirTol;

    bool m_bootDone;
    bool m_bootAccepted;
    bool m_locked;              // an estimate has been accepted
    unsigned int m_stillTotal;  // adapting samples so far (for the unlock)

    FusionVector m_bias;        // applied bias
    FusionVector m_anchor;      // last accepted estimate ...
    float m_anchorTemp;         // ... and its temperature

    // stillness episode
    unsigned int m_timer;
    bool m_active;
    FusionVector m_ref;         // gravity direction at the start of the episode
    FusionVector m_base;        // bias at the start of the adaptation

    bool m_tempValid;
    float m_temp;               // 1 s EMA of the die temperature
};
