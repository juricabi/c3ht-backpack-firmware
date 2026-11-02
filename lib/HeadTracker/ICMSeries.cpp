#include "Arduino.h"
#include "Wire.h"
#include "ICMSeries.h"

#define WHO_AM_I 0x75
#define REVISION_ID 0x01
#define ACCEL_X1 0x0B
#define INT_STATUS_DRDY 0x39

#if defined(HT_ICM_FS_1000DPS_4G)
// Head-tracking ranges (IMU_SELECTION.md F5): head motion stays well inside
// +-1000 deg/s and +-4 g, and the finer LSB (0.031 deg/s, 0.12 mg) is below
// the sensor noise instead of next to it.
#define ACCEL_CONFIG0_VAL 0x49      // ACCEL_UI_FS_SEL=2 (+-4 g), 100 Hz
#define GYRO_CONFIG0_VAL 0x29       // GYRO_UI_FS_SEL=1 (+-1000 dps), 100 Hz
#define ACCEL_RANGE 4.0
#define GYRO_RANGE 1000.0
#else
#define ACCEL_CONFIG0_VAL 9         // 16G, 100Hz
#define GYRO_CONFIG0_VAL 9          // 2000dps, 100Hz
#define ACCEL_RANGE 16.0
#define GYRO_RANGE 2000.0
#endif

#define ARES (ACCEL_RANGE / 32768)
#define GRES (GYRO_RANGE / 32768)

bool ICMSeries::initialize() {
    // Make sure it's one of the supported ICM IMUs
    // ICM42607P, ICM42607C, ICM42670P, ICM42670S, ICM42670T
    int whoami = readRegister(WHO_AM_I);
    if (whoami != 0x60 && whoami != 0x61 && whoami != 0x67 && whoami != 0x69 && whoami != 0x64) {
        return false;
    }

    /* Reset device */
    writeRegister(0x7C, 0);         // BLK_SEL_R
    writeRegister(0x79, 0);         // BLK_SEL_W
    writeRegister(0x02, 0x10);      // SIGNAL_PATH_RESET (soft reset device)
    delay(1);
    writeRegister(0x01, 0x04);      // DEVICE_CONFIG
    writeRegister(0x36, 0x41);      // INF_CONFIG1, i2c mode, PLL clock

    readRegister(0x3A);             // Clear the reset done interrupt

    writeRegister(0x06, 0x03);      // INT_CONFIG, interrupt push-pull, active high
    writeRegister(0x2B, 0x08);      // INT_SOURCE0, drdy

    writeRegister(0x21, ACCEL_CONFIG0_VAL);   // ACCEL_CONFIG0, range, 100Hz
    writeRegister(0x20, GYRO_CONFIG0_VAL);    // GYRO_CONFIG0, range, 100Hz
    writeRegister(0x23, 0x05);      // GYRO_CONFIG1, 34Hz UI LPF
    writeRegister(0x24, 0x45);      // ACCEL_CONFIG1, 34Hz UI LPF
    writeRegister(0x1F, 0x0F);      // Low noise mode for Accel/Gyro

    deltaTime = 0.01f;

    gyroRange = GYRO_RANGE;
    gRes = GRES;

    return true;
}

bool ICMSeries::getDataFromRegisters(FusionVector &accel, FusionVector &gyro) {
    uint8_t values[12];

    // return if NOT ready
    if (!(readRegister(INT_STATUS_DRDY) & 1)) {
        return false;
    }

    // read the accel and gyro data
    if (readBuffer(ACCEL_X1, values, 12) != 12) {
        return false;
    }

    // map into Gs and dps
    accel.axis.x =  (int16_t)((values[0] << 8) | values[1]) * ARES;
    accel.axis.y =  (int16_t)((values[2] << 8) | values[3]) * ARES;
    accel.axis.z =  (int16_t)((values[4] << 8) | values[5]) * ARES;
    gyro.axis.x =  (int16_t)((values[6] << 8) | values[7]) * GRES;
    gyro.axis.y =  (int16_t)((values[8] << 8) | values[9]) * GRES;
    gyro.axis.z =  (int16_t)((values[10] << 8) | values[11]) * GRES;
    return true;
}
