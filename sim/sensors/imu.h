#pragma once

#include "common/math/two_jet.h"
#include "common/matrix_defs.h"
#include "common/measurements/imu.h"
#include "common/out.h"
#include "common/utctime.h"

namespace mc {
namespace sim {
namespace sensors {

class ImuSim
{
 public:
    struct Options
    {
        double update_rate_hz;
        double accel_noise_stdev;
        double accel_walk_stdev;
        double gyro_noise_stdev;
        double gyro_walk_stdev;
    };

    ImuSim(const Options& options, const UTCTime& t0);

    bool sample(const UTCTime& t, const math::TwoJet<double>& x, Out<meas::ImuSample> imu);

    UTCTime prev_t_;
    Options options_;
    Vec3 accel_bias_;
    Vec3 gyro_bias_;
    Vec6 R_;
};

}  // namespace sensors
}  // namespace sim
}  // namespace mc
