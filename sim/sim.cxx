#include "sim/sim.h"

namespace mc {
namespace sim {

Sim::Sim(const Options& options)
    : options_(options),
      waypoint_follower_(options.wp_options),
      dynamics_(UTCTime(0), options.car_options),
      gnss_(options_.gnss_options, t()),
      imu_(options_.imu_options, t())
{
}

void Sim::step(const double dt)
{
    const Eigen::VectorXd control = waypoint_follower_.follow(dynamics_.x.x);
    const Eigen::VectorXd u =
        dynamics_.controller(t(), control) * randomNormal(u.rows()) * options_.input_noise;
    dynamics_.step(t() + dt, u);

    std::vector<meas::GnssObservation> obs;
    if (gnss_.sample(t(), dynamics_.x, Out(obs)) && gnss_callback_ != nullptr)
    {
        gnss_callback_(obs);
    }

    meas::ImuSample imu;
    if (imu_.sample(t(), dynamics_.x, Out(imu)) && imu_callback_ != nullptr)
    {
        imu_callback_(imu);
    }
}

}  // namespace sim
}  // namespace mc
