#pragma once

#include "common/matrix_defs.h"
#include "common/utctime.h"

namespace mc {
namespace meas {

struct ImuSample
{
    UTCTime t;
    Vec3 accel;
    Vec3 gyro;

    void setZero()
    {
        accel.setZero();
        gyro.setZero();
        t.sec = 0;
        t.nsec = 0;
    }

    void setRandom()
    {
        accel.setRandom();
        gyro.setRandom();
        t.sec = 0;
        t.nsec = 0;
    }
};

}  // namespace meas
}  // namespace mc
