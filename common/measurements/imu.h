#pragma once

#include "common/matrix_defs.h"
#include "common/utctime.h"

namespace mc {
namespace meas {

class ImuSample
{
 public:
    UTCTime t;
    Vec3 accel;
    Vec3 gyro;

    static ImuSample Zero()
    {
        ImuSample out;
        out.setZero();
        return out;
    }

    static ImuSample Random()
    {
        ImuSample out;
        out.setRandom();
        return out;
    }

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
