#include "sim/sensors/imu.h"

#include "common/random.h"

namespace mc {
namespace sim {
namespace sensors {

ImuSim::ImuSim(const ImuSim::Options& options, const UTCTime& t0) : options_(options)
{
    prev_t_ = t0;
    accel_bias_ = options_.accel_walk_stdev * Vec3::Random();
    gyro_bias_ = options_.gyro_walk_stdev * Vec3::Random();
    R_.head<3>().setConstant(options.accel_noise_stdev * options.accel_noise_stdev);
    R_.tail<3>().setConstant(options.gyro_noise_stdev * options.gyro_noise_stdev);
}

bool ImuSim::sample(const UTCTime& t, const math::TwoJet<double>& x, Out<meas::ImuSample> imu)
{
    const double dt = (t - prev_t_).toSec();
    if (dt >= 1.0 / options_.update_rate_hz)
    {
        prev_t_ = t;

        accel_bias_ += randomNormal<Vec3>() * options_.accel_noise_stdev * dt;
        gyro_bias_ += randomNormal<Vec3>() * options_.gyro_noise_stdev * dt;

        imu->accel = x.d2x.linear() + accel_bias_ +
                     randomNormal<Vec3>() * options_.accel_noise_stdev - x.x.rota(GRAVITY);
        imu->gyro = x.dx.angular() + gyro_bias_ + randomNormal<Vec3>() * options_.gyro_noise_stdev;
        imu->t = t;
        return true;
    }
    return false;
}

}  // namespace sensors
}  // namespace sim
}  // namespace mc
