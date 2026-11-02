#ifndef BACKPACK_IMUBASE_H
#define BACKPACK_IMUBASE_H

#include "FusionMath.h"

class IMUBase {
public:
    virtual ~IMUBase() = default;

    virtual bool initialize() = 0;
    void setInterruptHandler(int pin);

    bool readIMUData(FusionVector &accel, FusionVector &gyro);

    void beginCalibration();
    bool updateCalibration(FusionVector &g);

    void setCalibration(float (*calibration)[3]);

    const float *getCalibration();

    float getDeltaTime() const { return deltaTime; }
    // micros() of the data-ready interrupt of the sample last returned by readIMUData()
    uint32_t getSampleTimeUs() const { return sampleTimeUs; }
    float getGyroRange() const { return gyroRange; }

protected:
    int address;
    float deltaTime = 0.0f;
    float gyroRange;
    float gRes = 0;
    uint32_t sampleTimeUs = 0;

    explicit IMUBase(int _address) : address(_address) {}

    void writeRegister(uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t reg);

    uint8_t readBuffer(uint8_t reg, uint8_t *buffer, int length);

    virtual bool getDataFromRegisters(FusionVector &accel, FusionVector &gyro) = 0;
};

#endif //BACKPACK_IMUBASE_H
