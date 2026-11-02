#include "GyroBias.h"

#include <math.h>

GyroBiasSettings GyroBiasDefaultSettings(unsigned int sampleRateHz)
{
    if (sampleRateHz == 0)
    {
        sampleRateHz = 100;
    }
    GyroBiasSettings s;
    s.sampleRateHz = sampleRateHz;

    s.bootDiscardS = 0.2f;          // gyro start-up 30 ms + LPF settling
    s.bootGateS = 1.0f;
    s.bootMeanS = 2.0f;             // std error ~0.07 / sqrt(200) = 0.005 deg/s
    s.bootTimeoutS = 10.0f;
    s.stillStdDps = 0.15f;          // datasheet total noise 0.07 deg/s rms
    s.maxBootBiasDps = 3.0f;

    s.accelNormTolG = 0.03f;
    s.accelNormAbsTolG = 0.15f;
    s.accelDirTolDeg = 0.5f;

    s.thresholdDps = 1.0f;
    s.thresholdUnlockedDps = 3.0f;  // FusionOffset default
    s.timeoutS = 3.0f;
    s.cutoffHz = 0.02f;             // FusionOffset default, time constant ~8 s
    s.unlockStillS = 25.0f;         // ~3 time constants
    s.stepClampDps = 0.05f;
    s.stepClampDpsPerK = 0.02f;
    s.maxBiasDps = 5.0f;
    return s;
}

static unsigned int samples(float seconds, unsigned int rate)
{
    if (seconds <= 0.0f)
    {
        return 0;
    }
    return (unsigned int)(seconds * (float)rate + 0.5f);
}

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

// Accel part of the stillness test. The norm is compared with the norm at the
// start of the window/episode, not with 1 g: the ICM-42670-P allows +-25 mg
// zero-g offset and +-1 % sensitivity per axis, so a resting board can read
// 1 g +- ~0.04 g and an absolute 0.03 g gate would never pass on such a unit.
// A loose absolute check (absTol) still rejects free fall and hard shaking.
static bool accelGate(const FusionVector &accel, const FusionVector &ref, bool hasRef, float normTol, float absTol, float cosDirTol)
{
    const float n = sqrtf(accel.axis.x * accel.axis.x + accel.axis.y * accel.axis.y + accel.axis.z * accel.axis.z);
    if (!(fabsf(n - 1.0f) <= absTol))      // also rejects NaN
    {
        return false;
    }
    if (!hasRef)
    {
        return true;
    }
    const float nr = sqrtf(ref.axis.x * ref.axis.x + ref.axis.y * ref.axis.y + ref.axis.z * ref.axis.z);
    if (!(nr > 0.0f) || !(fabsf(n - nr) <= normTol))
    {
        return false;
    }
    const float c = (accel.axis.x * ref.axis.x + accel.axis.y * ref.axis.y + accel.axis.z * ref.axis.z) / (n * nr);
    return c >= cosDirTol;
}

// ---------------------------------------------------------------- StillCalibrator

void StillCalibrator::begin(const GyroBiasSettings &s)
{
    const unsigned int rate = s.sampleRateHz ? s.sampleRateHz : 100;
    m_discardN = samples(s.bootDiscardS, rate);
    m_gateN = samples(s.bootGateS, rate);
    m_meanN = samples(s.bootMeanS, rate);
    m_timeoutN = samples(s.bootTimeoutS, rate);
    if (m_gateN < 2) m_gateN = 2;
    if (m_meanN < 2) m_meanN = 2;
    m_stillStd = s.stillStdDps;
    m_maxBias = s.maxBootBiasDps;
    m_normTol = s.accelNormTolG;
    m_normAbsTol = s.accelNormAbsTolG;
    m_cosDirTol = cosf(s.accelDirTolDeg * 0.017453292f);

    m_phase = DISCARD;
    m_result = RUNNING;
    m_seen = 0;
    m_mean = FUSION_VECTOR_ZERO;
    m_ref = FUSION_VECTOR_ZERO;
    restartWindow(DISCARD, FUSION_VECTOR_ZERO);
}

void StillCalibrator::restartWindow(Phase phase, const FusionVector &accel)
{
    m_phase = phase;
    m_n = 0;
    for (int i = 0; i < 3; i++)
    {
        m_wMean[i] = 0.0f;
        m_wM2[i] = 0.0f;
    }
    if (phase == GATE)
    {
        m_ref = accel;
    }
}

StillCalibrator::Result StillCalibrator::update(const FusionVector &gyro, const FusionVector &accel)
{
    if (m_phase == DONE)
    {
        return m_result;
    }
    if (m_phase == DISCARD)
    {
        if (m_seen < m_discardN)
        {
            m_seen++;
            return RUNNING;
        }
        m_seen = 0;
        restartWindow(GATE, accel);
    }

    m_seen++;
    if (m_phase == GATE && m_seen >= m_timeoutN && m_timeoutN > 0)
    {
        m_phase = DONE;
        m_result = TIMED_OUT;
        return m_result;
    }

    if (m_n == 0 && m_phase == GATE)
    {
        m_ref = accel;
    }
    if (!accelGate(accel, m_ref, true, m_normTol, m_normAbsTol, m_cosDirTol))
    {
        restartWindow(GATE, accel);
        return RUNNING;
    }

    // Welford running mean / variance
    m_n++;
    for (int i = 0; i < 3; i++)
    {
        const float x = gyro.array[i];
        const float d = x - m_wMean[i];
        m_wMean[i] += d / (float)m_n;
        m_wM2[i] += d * (x - m_wMean[i]);
    }

    const unsigned int need = m_phase == GATE ? m_gateN : m_meanN;
    if (m_n < need)
    {
        return RUNNING;
    }

    bool quiet = true;
    for (int i = 0; i < 3; i++)
    {
        const float sd = sqrtf(m_wM2[i] / (float)(m_n - 1));
        if (!(sd < m_stillStd))
        {
            quiet = false;
        }
    }
    if (!quiet)
    {
        restartWindow(GATE, accel);
        return RUNNING;
    }
    if (m_phase == GATE)
    {
        restartWindow(MEAN, accel);     // keeps the gravity reference
        return RUNNING;
    }

    // MEAN window complete and still
    bool sane = true;
    for (int i = 0; i < 3; i++)
    {
        m_mean.array[i] = m_wMean[i];
        if (!(fabsf(m_wMean[i]) <= m_maxBias))
        {
            sane = false;
        }
    }
    m_phase = DONE;
    m_result = sane ? ACCEPTED : REJECTED;
    return m_result;
}

// ---------------------------------------------------------------- GyroBiasEstimator

void GyroBiasEstimator::begin(const GyroBiasSettings &settings)
{
    m_settings = settings;
    if (m_settings.sampleRateHz == 0)
    {
        m_settings.sampleRateHz = 100;
    }
    m_settings.maxBiasDps = fabsf(m_settings.maxBiasDps);
    const unsigned int rate = m_settings.sampleRateHz;
    m_k = 2.0f * 3.14159265f * m_settings.cutoffHz / (float)rate;
    m_timeoutN = samples(m_settings.timeoutS, rate);
    m_unlockN = samples(m_settings.unlockStillS, rate);
    m_cosDirTol = cosf(m_settings.accelDirTolDeg * 0.017453292f);

    m_boot.begin(m_settings);
    m_bootDone = m_settings.bootMeanS <= 0.0f;
    m_bootAccepted = false;
    m_locked = false;
    m_stillTotal = 0;

    m_bias = FUSION_VECTOR_ZERO;
    m_anchor = FUSION_VECTOR_ZERO;
    m_anchorTemp = 0.0f;
    m_cal = FUSION_VECTOR_ZERO;
    m_calTemp = 0.0f;
    m_calTempValid = false;

    m_timer = 0;
    m_active = false;
    m_ref = FUSION_VECTOR_ZERO;
    m_base = FUSION_VECTOR_ZERO;

    m_tempValid = false;
    m_temp = 0.0f;
    m_model.reset();
}

void GyroBiasEstimator::restartCalibrated(float temperatureC)
{
    begin(m_settings);
    updateTemperature(temperatureC);
    m_bootDone = true;
    m_bootAccepted = true;
    acceptEstimate();
}

void GyroBiasEstimator::updateTemperature(float temperatureC)
{
    if (!(temperatureC > -60.0f && temperatureC < 150.0f))     // NaN or nonsense
    {
        return;
    }
    if (!m_tempValid)
    {
        m_tempValid = true;
        m_temp = temperatureC;
        return;
    }
    const float alpha = 1.0f / (float)m_settings.sampleRateHz;    // 1 s EMA
    m_temp += alpha * (temperatureC - m_temp);
}

void GyroBiasEstimator::clampBias(FusionVector &b) const
{
    for (int i = 0; i < 3; i++)
    {
        b.array[i] = clampf(b.array[i], -m_settings.maxBiasDps, m_settings.maxBiasDps);
    }
}

// An estimate is accepted: it becomes the reference for the step clamp and
// the value applied between stillness episodes. The first one (boot or web
// calibration, or the unlocked estimate once converged) is also the centre of
// the envelope (envelopeLimit()) for everything that follows.
void GyroBiasEstimator::acceptEstimate()
{
    if (!m_locked)
    {
        m_cal = m_bias;
        m_calTemp = m_temp;
        m_calTempValid = m_tempValid;
    }
    m_anchor = m_bias;
    m_anchorTemp = m_temp;
    m_locked = true;
    if (m_tempValid)
    {
        m_model.add(m_temp, m_bias);
    }
}

// Bias to apply while not adapting: the last estimate, moved along the
// learned bias-vs-temperature curve to the current temperature.
FusionVector GyroBiasEstimator::modelBias() const
{
    if (!m_tempValid)
    {
        return m_anchor;
    }
    return FusionVectorAdd(m_anchor, m_model.delta(m_anchorTemp, m_temp));
}

// Largest |bias - calibration| allowed while locked. A slow steady head turn
// below the stillness threshold looks exactly like bias; measured per episode
// it could add one step clamp per episode without limit. Measured from the
// calibration it stays below stepClampDps plus what the temperature change
// since the calibration can explain.
float GyroBiasEstimator::envelopeLimit() const
{
    float lim = m_settings.stepClampDps;
    if (m_calTempValid && m_tempValid)
    {
        lim += m_settings.stepClampDpsPerK * fabsf(m_temp - m_calTemp);
    }
    return lim;
}

void GyroBiasEstimator::clampEnvelope(FusionVector &b) const
{
    if (!m_locked)
    {
        return;
    }
    const float lim = envelopeLimit();
    for (int i = 0; i < 3; i++)
    {
        b.array[i] = clampf(b.array[i], m_cal.array[i] - lim, m_cal.array[i] + lim);
    }
}

bool GyroBiasEstimator::accelStill(const FusionVector &accel) const
{
    return accelGate(accel, m_ref, m_timer > 0, m_settings.accelNormTolG, m_settings.accelNormAbsTolG, m_cosDirTol);
}

FusionVector GyroBiasEstimator::update(FusionVector gyro, const FusionVector &accel, float temperatureC)
{
    updateTemperature(temperatureC);

    if (!m_bootDone)
    {
        const StillCalibrator::Result r = m_boot.update(gyro, accel);
        if (r == StillCalibrator::ACCEPTED)
        {
            m_bias = m_boot.mean();
            clampBias(m_bias);
            m_bootAccepted = true;
            m_bootDone = true;
            acceptEstimate();
        }
        else if (r != StillCalibrator::RUNNING)
        {
            m_bootDone = true;      // not still, or implausible: the runtime estimate takes over
        }
        return FusionVectorSubtract(gyro, m_bias);
    }

    if (!m_active)
    {
        m_bias = modelBias();
        clampEnvelope(m_bias);
        clampBias(m_bias);
    }

    const FusionVector corrected = FusionVectorSubtract(gyro, m_bias);
    const float threshold = m_locked ? m_settings.thresholdDps : m_settings.thresholdUnlockedDps;
    bool still = fabsf(corrected.axis.x) < threshold && fabsf(corrected.axis.y) < threshold && fabsf(corrected.axis.z) < threshold;
    if (still)
    {
        still = accelStill(accel);
    }

    if (!still)
    {
        if (m_active)
        {
            // end of an adapting episode: keep what it learned; once
            // converged (locked) it is also a point of the temperature curve
            if (m_locked)
            {
                acceptEstimate();
            }
            else
            {
                m_anchor = m_bias;
                m_anchorTemp = m_temp;
            }
        }
        m_active = false;
        m_timer = 0;
        return corrected;
    }

    if (m_timer == 0)
    {
        m_ref = accel;
    }
    if (m_timer < m_timeoutN)
    {
        m_timer++;
        return corrected;
    }

    if (!m_active)
    {
        m_active = true;
        m_base = m_bias;
    }
    m_stillTotal++;

    float lim = m_settings.maxBiasDps;
    if (m_locked)
    {
        lim = m_settings.stepClampDps;
        if (m_tempValid)
        {
            lim += m_settings.stepClampDpsPerK * fabsf(m_temp - m_anchorTemp);
        }
    }
    for (int i = 0; i < 3; i++)
    {
        float b = m_bias.array[i] + m_k * (gyro.array[i] - m_bias.array[i]);
        b = clampf(b, m_base.array[i] - lim, m_base.array[i] + lim);
        m_bias.array[i] = b;
    }
    clampEnvelope(m_bias);
    clampBias(m_bias);

    if (!m_locked && m_stillTotal >= m_unlockN)
    {
        acceptEstimate();           // converged: from now on the tight threshold and step clamp
        m_base = m_bias;
    }
    return FusionVectorSubtract(gyro, m_bias);
}
