#include <Arduino.h>

#include <Wire.h>

#include "crsf_protocol.h"

#include "devHeadTracker.h"

#if defined(HAS_HEADTRACKING)
#include "config.h"
#include "logging.h"

#include "IMUBase.h"
#include "ICMSeries.h"
#include "MPU6050.h"
#include "QMI8658C.h"
#include "Fusion.h"
#if defined(HT_GYRO_BIAS_TRACKING)
#include "GyroBias.h"
#endif
#if defined(HT_MEASURED_SAMPLE_PERIOD)
#include "SamplePeriod.h"
static SamplePeriod samplePeriod;
#endif

static HeadTrackerState ht_state = STATE_ERROR;
static IMUBase *imu;
static bool hasCompass = false;
static FusionAhrs ahrs;

static float orientation[3] = {0.0, 0.0, 0.0};
static FusionEuler euler;
static float rollHome = 0, pitchHome = 0, yawHome = 0;
static int calibrationData[3][2];
static uint32_t cal_started;
// set by startIMUCalibration() (web server task), handled in timeout()
static volatile bool calRequested = false;

#if defined(HT_GYRO_BIAS_TRACKING)
// Run-time gyro bias (boot calibration + stillness-gated runtime estimate),
// RAM only: nothing is persisted, so there is no flash wear however often it
// adapts.
static GyroBiasEstimator gyroBias;
static unsigned int gyroBiasRate = 0;
static StillCalibrator webCalibration;     // web-UI "ci"

static unsigned int imuRateHz()
{
    return (unsigned int)((1.0f / imu->getDeltaTime()) + 0.5f);
}

static void gyroBiasBegin()
{
    gyroBiasRate = imuRateHz();
    gyroBias.begin(GyroBiasDefaultSettings(gyroBiasRate));
}

// After an IMU re-init keep what was learned, unless a different IMU type
// (data rate) came back.
static void gyroBiasOnline()
{
    if (gyroBiasRate != imuRateHz())
    {
        gyroBiasBegin();
    }
}
#endif

// IMU health: re-probe when no sample arrived for this long while running
// (100 Hz ODR, so 50 samples), and retry a failed probe with a growing delay.
#define IMU_STALL_TIMEOUT_MS 500
#define IMU_RETRY_MIN_MS 1000
#define IMU_RETRY_MAX_MS 30000
static uint32_t lastSampleMs;
static uint32_t retryDelayMs = IMU_RETRY_MIN_MS;

// I2C bus recovery (UM10204 3.1.16): a slave that was mid-transfer when the
// master reset or browned out keeps driving SDA low and the bus is dead until
// it is clocked out. Nine SCL pulses with SDA released finish any byte plus
// its ACK slot, then a STOP returns every slave to idle. Harmless on an idle
// bus. Must run while the pins are not attached to the I2C peripheral.
static void i2cBusClear()
{
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(5);
    for (int i = 0; i < 9; i++)
    {
        digitalWrite(PIN_SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(PIN_SCL, HIGH);
        delayMicroseconds(5);
    }
    // STOP: SDA low -> high while SCL is high
    digitalWrite(PIN_SCL, LOW);
    pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(PIN_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_SDA, HIGH);
    delayMicroseconds(5);
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);
}

static void i2cBegin()
{
    i2cBusClear();
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    Wire.setTimeout(1000);
}

static void i2cRestart()
{
#if defined(PLATFORM_ESP32)
    Wire.end();     // release the pins so the bus clear can drive them
#endif
    i2cBegin();
}

// Probe the supported IMUs; on success imu is set up and its interrupt attached.
static bool probeIMU()
{
    // ICMSeries first: it matches an exact WHO_AM_I list, while the MPU6050
    // check masks bit 0 and would also take an ICM-42670-S (0x69) for an MPU6050.
    imu = new ICMSeries();
    if (imu->initialize()) {
        DBGLN("Found ICM Series IMU")
    }
    else {
        delete imu;
        imu = new MPU6050();
        if (imu->initialize()) {
            DBGLN("Found MPU6050 IMU")
        }
        else {
            delete imu;
            imu = new QMI8658C();
            if (imu->initialize()) {
                DBGLN("Found QMI8658C IMU")
            }
            else {
                delete imu;
                imu = nullptr;
                return false;
            }
        }
    }
    if (imu->getDeltaTime() <= 0.0f) {
        delete imu;
        imu = nullptr;
        return false;
    }
    imu->setInterruptHandler(PIN_INT);

    // Set AHRS algorithm settings (they depend on the IMU type)
    const unsigned int recoveryTriggerPeriod = (unsigned int) ((5.0f / imu->getDeltaTime()) + 0.5f);
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = imu->getGyroRange(), /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = recoveryTriggerPeriod,
    };
    FusionAhrsSetSettings(&ahrs, &settings);
    return true;
}

static void dropIMU()
{
    detachInterrupt(PIN_INT);
    delete imu;
    imu = nullptr;
}

// Called with the config loaded, after a (re-)probe found the IMU.
static void imuOnline()
{
    imu->setCalibration(config.GetIMUCalibration());
#if defined(HT_GYRO_BIAS_TRACKING)
    gyroBiasOnline();
#endif
#if defined(HT_MEASURED_SAMPLE_PERIOD)
    samplePeriod.begin(imu->getDeltaTime());
#endif
    lastSampleMs = millis();
    retryDelayMs = IMU_RETRY_MIN_MS;
    ht_state = STATE_RUNNING;
}

static void initialize()
{
    FusionAhrsInitialise(&ahrs);
    i2cBegin();

    // Initializing the IMU, with a short backoff: right after power-up the
    // IMU may still be in its start-up time, or the slot rail still ramping.
    static const uint16_t backoffMs[] = {0, 10, 50, 200};
    for (unsigned int i = 0; i < sizeof(backoffMs) / sizeof(backoffMs[0]); i++)
    {
        if (backoffMs[i]) {
            delay(backoffMs[i]);
            i2cRestart();
        }
        if (probeIMU()) {
            DBGLN("starting head tracker");
            ht_state = STATE_RUNNING;
            return;
        }
    }
    DBGLN("no IMU found, retrying later");
    ht_state = STATE_ERROR;
}

static int start()
{
    memcpy(orientation, *config.GetBoardOrientation(), sizeof(orientation));

#ifdef DEBUG_LOG
    Serial.printf("Load %7.2f %7.2f %7.2f\r\n", orientation[0], orientation[1], orientation[2]);
#endif
    if (isnan(orientation[0]) || isnan(orientation[1]) || isnan(orientation[2])) {
        orientation[0] = 0;
        orientation[1] = 0;
        orientation[2] = 0;
    }
    FusionAhrsReset(&ahrs);
    if (imu == nullptr)
    {
        ht_state = STATE_ERROR;
        return retryDelayMs;
    }
    imuOnline();
    return DURATION_IMMEDIATELY;
}

// The IMU stopped producing samples (e.g. it was reset by a supply dip and
// lost its configuration, or it holds SDA low) or was never found: clear
// the bus and probe again, backing off while it keeps failing. The AHRS
// state, orientation and centre are kept so a recovered IMU carries on,
// but the output angles are centred meanwhile: a gimbal must not stay
// frozen at the last head angle for the whole retry period (up to 30 s
// per attempt), and "sc" in that time must not move the centre.
static int recoverIMU()
{
    if (imu != nullptr)
    {
        DBGLN("IMU stalled, re-initialising");
        dropIMU();
    }
    ht_state = STATE_ERROR;
    euler.angle.roll = 0.0f;
    euler.angle.pitch = 0.0f;
    euler.angle.yaw = 0.0f;
    i2cRestart();
    if (probeIMU())
    {
        DBGLN("IMU recovered");
        imuOnline();
        return DURATION_IMMEDIATELY;
    }
    const int wait = retryDelayMs;
    retryDelayMs = retryDelayMs * 2 > IMU_RETRY_MAX_MS ? IMU_RETRY_MAX_MS : retryDelayMs * 2;
    return wait;
}

// Normalizes any number to an arbitrary range
// by assuming the range wraps around when going below min or above max
static float normalize(float value, float start, float end) {
    float width = end - start;         //
    float offsetValue = value - start; // value relative to 0

    return (offsetValue - (floor(offsetValue / width) * width)) + start;
    // + start to reset back to start of original range
}

// Rotate a point (pn) in space in Order X -> Y -> Z
static void rotate(float pn[3], const float rot[3]) {
    float out[3];

    // X-axis Rotation
    if (rot[0] != 0) {
        out[0] = pn[0] * 1 + pn[1] * 0 + pn[2] * 0;
        out[1] = pn[0] * 0 + pn[1] * cos(rot[0]) - pn[2] * sin(rot[0]);
        out[2] = pn[0] * 0 + pn[1] * sin(rot[0]) + pn[2] * cos(rot[0]);
        memcpy(pn, out, sizeof(out[0]) * 3);
    }

    // Y-axis Rotation
    if (rot[1] != 0) {
        out[0] = pn[0] * cos(rot[1]) - pn[1] * 0 + pn[2] * sin(rot[1]);
        out[1] = pn[0] * 0 + pn[1] * 1 + pn[2] * 0;
        out[2] = -pn[0] * sin(rot[1]) + pn[1] * 0 + pn[2] * cos(rot[1]);
        memcpy(pn, out, sizeof(out[0]) * 3);
    }

    // Z-axis Rotation
    if (rot[2] != 0) {
        out[0] = pn[0] * cos(rot[2]) - pn[1] * sin(rot[2]) + pn[2] * 0;
        out[1] = pn[0] * sin(rot[2]) + pn[1] * cos(rot[2]) + pn[2] * 0;
        out[2] = pn[0] * 0 + pn[1] * 0 + pn[2] * 1;
        memcpy(pn, out, sizeof(out[0]) * 3);
    }
}

static void beginIMUCalibration();

static int timeout()
{
    FusionVector a;
    FusionVector g;

    if (calRequested) {
        calRequested = false;   // ignored while there is no IMU
        beginIMUCalibration();
    }
    if (imu == nullptr || ht_state == STATE_ERROR) {
        return recoverIMU();
    }
    if (!imu->readIMUData(a, g)) {
        if (millis() - lastSampleMs > IMU_STALL_TIMEOUT_MS) {
            return recoverIMU();
        }
        return DURATION_IMMEDIATELY;
    }
    lastSampleMs = millis();
#if defined(HT_MEASURED_SAMPLE_PERIOD)
    const float deltaTime = samplePeriod.update(imu->getSampleTimeUs());
#if defined(DEBUG_LOG)
    static uint32_t lastMissed = 0, lastMissedLogMs = 0;
    if (samplePeriod.missedSamples() != lastMissed && lastSampleMs - lastMissedLogMs > 10000) {
        lastMissed = samplePeriod.missedSamples();
        lastMissedLogMs = lastSampleMs;
        DBGLN("IMU period %d us, missed %u samples in %u gaps", (int)(samplePeriod.period() * 1e6f),
              (unsigned)samplePeriod.missedSamples(), (unsigned)samplePeriod.gaps());
    }
#endif
#else
    const float deltaTime = imu->getDeltaTime();
#endif

    switch (ht_state) {
        case STATE_RUNNING: {
#if defined(HT_GYRO_BIAS_TRACKING)
            // sensor frame, after the stored calibration, before the board rotation
            g = gyroBias.update(g, a, imu->getTemperature());
#endif
            rotate(a.array, orientation);
            rotate(g.array, orientation);

            FusionAhrsUpdate(&ahrs, g, a, FUSION_VECTOR_ZERO, deltaTime);

            euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
            euler.angle.roll -= rollHome;
            euler.angle.pitch -= pitchHome;
            euler.angle.yaw -= yawHome;
            break;
        }

        case STATE_IMU_CALIBRATING: {
#if defined(HT_GYRO_BIAS_TRACKING)
            // mean of a still window (F1), saved only if the stillness tests pass
            const StillCalibrator::Result r = webCalibration.update(g, a);
            if (r == StillCalibrator::ACCEPTED) {
                const float *cal = imu->getCalibration();
                const FusionVector m = webCalibration.mean();
                float newCal[3] = {cal[0] - m.axis.x, cal[1] - m.axis.y, cal[2] - m.axis.z};
                imu->setCalibration(&newCal);
                config.SetIMUCalibration(imu->getCalibration());
                config.Commit();
                gyroBias.restartCalibrated(imu->getTemperature());
                DBGLN("IMU calibration saved");
                ht_state = STATE_RUNNING;
            } else if (r != StillCalibrator::RUNNING) {
                DBGLN("IMU calibration failed (not still), stored calibration kept");
                ht_state = STATE_RUNNING;
            }
#else
            if (imu->updateCalibration(g)) {
                config.SetIMUCalibration(imu->getCalibration());
                config.Commit();
                ht_state = STATE_RUNNING;
            }
#endif
            break;
        }

        default:
            break;
    }

    return DURATION_IMMEDIATELY;
}

// Runs in the device loop (timeout()), never in the web server task: the
// IMU object may be deleted and re-created there by recoverIMU().
static void beginIMUCalibration()
{
    if (imu == nullptr || ht_state == STATE_ERROR)
    {
        return;
    }
#if defined(HT_GYRO_BIAS_TRACKING)
    GyroBiasSettings s = GyroBiasDefaultSettings(imuRateHz());
    // this sets the stored calibration itself, so allow any plausible raw
    // MEMS offset instead of the 3 deg/s limit for the residual at boot
    s.maxBootBiasDps = 30.0f;
    webCalibration.begin(s);
#else
    imu->beginCalibration();
#endif
    cal_started = millis();
    ht_state = STATE_IMU_CALIBRATING;
}

// Called from the web server (AsyncTCP) task: only flag the request.
void startIMUCalibration()
{
    calRequested = true;
}

HeadTrackerState getHeadTrackerState()
{
    return ht_state;
}

void resetBoardOrientation()
{
    orientation[0] = 0;
    orientation[1] = 0;
    orientation[2] = 0;
    rollHome = 0;
    pitchHome = 0;
    yawHome = 0;
    FusionAhrsReset(&ahrs);
}

void saveBoardOrientation()
{
#ifdef DEBUG_LOG
    Serial.printf("Save %7.2f %7.2f %7.2f\r\n", orientation[0], orientation[1], orientation[2]);
#endif
    config.SetBoardOrientation(orientation);
    config.Commit();
}

void setBoardOrientation(int xAngle, int yAngle, int zAngle)
{
    orientation[0] = xAngle * DEG_TO_RAD;
    orientation[1] = yAngle * DEG_TO_RAD;
    orientation[2] = zAngle * DEG_TO_RAD;
    ahrs.initialising = true;
    ahrs.rampedGain = 10.0f;
}

void resetCenter()
{
    rollHome += euler.angle.roll;
    pitchHome += euler.angle.pitch;
    yawHome += euler.angle.yaw;
    rollHome = normalize(rollHome, -180.0, 180.0);
    pitchHome = normalize(pitchHome, -180.0, 180.0);
    yawHome = normalize(yawHome, -180.0, 180.0);
}

void getEuler(float *yaw, float *pitch, float *roll)
{
    *yaw = normalize(euler.angle.yaw, -180.0, 180.0);
    *pitch = normalize(euler.angle.pitch, -180.0, 180.0);
    *roll = normalize(euler.angle.roll, -180.0, 180.0);
}

device_t HeadTracker_device = {
    .initialize = initialize,
    .start = start,
    .event = nullptr,
    .timeout = timeout
};

#endif
#if defined(SUPPORT_HEADTRACKING)
extern int16_t ptrChannelData[3];

HeadTrackerState getHeadTrackerState()
{
    return STATE_RUNNING;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    const float run = in_max - in_min;
    const float rise = out_max - out_min;
    const float delta = x - in_min;
    return (delta * rise) / run + out_min;
}

void resetCenter()
{
}

void getEuler(float *yaw, float *pitch, float *roll)
{
    *yaw = -fmap(ptrChannelData[0], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, -180.0, 180.0);
    *pitch = fmap(ptrChannelData[2], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, -180.0, 180.0);
    *roll = fmap(ptrChannelData[1], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, -180.0, 180.0);
}
#endif
