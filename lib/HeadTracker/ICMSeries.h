#ifndef BACKPACK_ICM_H
#define BACKPACK_ICM_H

#include "FusionMath.h"
#include "IMUBase.h"

extern uint8_t c3htImuWhoAmI;      // last WHO_AM_I read by ICMSeries::initialize()

class ICMSeries : public IMUBase {
public:
    ICMSeries() : IMUBase(0x68) {}
    bool initialize();

protected:
    bool getDataFromRegisters(FusionVector &accel, FusionVector &gyro);

private:
    void writeMem1Register(uint8_t reg, uint8_t val);
};

#endif //BACKPACK_ICM_H
