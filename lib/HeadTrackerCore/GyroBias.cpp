#include "GyroBias.h"

GyroBiasSettings GyroBiasDefaultSettings(unsigned int sampleRateHz)
{
    if (sampleRateHz == 0)
    {
        sampleRateHz = 100;
    }
    GyroBiasSettings s;
    s.sampleRateHz = sampleRateHz;
    s.bootSamples = 2 * sampleRateHz;
    s.bootTimeoutSamples = 15 * sampleRateHz;
    s.bootStillP2pDps = 1.0f;
    s.maxBiasDps = 5.0f;
    return s;
}

static float clampf(float v, float lim)
{
    if (v > lim) return lim;
    if (v < -lim) return -lim;
    return v;
}

void GyroBiasEstimator::begin(const GyroBiasSettings &settings)
{
    m_settings = settings;
    if (m_settings.sampleRateHz == 0)
    {
        m_settings.sampleRateHz = 100;
    }
    if (m_settings.maxBiasDps < 0.0f)
    {
        m_settings.maxBiasDps = -m_settings.maxBiasDps;
    }
    FusionOffsetInitialise(&m_offset, m_settings.sampleRateHz);
    m_bootDone = m_settings.bootSamples == 0;
    m_bootAccepted = false;
    m_bootSeen = 0;
    m_windowCount = 0;
}

void GyroBiasEstimator::restartWindow(const FusionVector &gyro)
{
    for (int i = 0; i < 3; i++)
    {
        m_sum[i] = gyro.array[i];
        m_min[i] = gyro.array[i];
        m_max[i] = gyro.array[i];
    }
    m_windowCount = 1;
}

void GyroBiasEstimator::bootSample(const FusionVector &gyro)
{
    m_bootSeen++;
    if (m_windowCount == 0)
    {
        restartWindow(gyro);
    }
    else
    {
        bool still = true;
        for (int i = 0; i < 3; i++)
        {
            const float lo = gyro.array[i] < m_min[i] ? gyro.array[i] : m_min[i];
            const float hi = gyro.array[i] > m_max[i] ? gyro.array[i] : m_max[i];
            if (hi - lo > m_settings.bootStillP2pDps)
            {
                still = false;
                break;
            }
        }
        if (still)
        {
            for (int i = 0; i < 3; i++)
            {
                m_sum[i] += gyro.array[i];
                if (gyro.array[i] < m_min[i]) m_min[i] = gyro.array[i];
                if (gyro.array[i] > m_max[i]) m_max[i] = gyro.array[i];
            }
            m_windowCount++;
        }
        else
        {
            restartWindow(gyro);    // moved: start a new window at this sample
        }
    }

    if (m_windowCount >= m_settings.bootSamples)
    {
        for (int i = 0; i < 3; i++)
        {
            m_offset.gyroscopeOffset.array[i] = m_sum[i] / (float)m_windowCount;
        }
        clampBias();
        m_offset.timer = 0;
        m_bootAccepted = true;
        m_bootDone = true;
    }
    else if (m_bootSeen >= m_settings.bootTimeoutSamples)
    {
        m_bootDone = true;      // never still long enough; FusionOffset carries on alone
    }
}

void GyroBiasEstimator::clampBias()
{
    for (int i = 0; i < 3; i++)
    {
        m_offset.gyroscopeOffset.array[i] = clampf(m_offset.gyroscopeOffset.array[i], m_settings.maxBiasDps);
    }
}

FusionVector GyroBiasEstimator::update(FusionVector gyro)
{
    if (!m_bootDone)
    {
        bootSample(gyro);
    }
    FusionVector corrected = FusionOffsetUpdate(&m_offset, gyro);
    clampBias();
    return corrected;
}
