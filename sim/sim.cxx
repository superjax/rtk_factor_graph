#include "sim/sim.h"

namespace mc {
namespace sim {

Sim::Sim(const Options& options)
    : options_(options),
      waypoint_follower_(options.wp_options),
      dynamics_(options.car_options),
      gnss_(options_.gnss_options, t()),
      imu_(options_.imu_options, t())
{
}

void Sim::step(const double dt)
{
    const Eigen::VectorXd control = waypoint_follower_.follow(dynamics_.x.x);
    const Eigen::VectorXd u =
        dynamics_.controller(t(), control) + randomNormal(2) * options_.input_noise;
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

    ephemeris::GPSEphemeris gps_eph;
    if (gnss_.sampleGpsEph(t(), make_out(gps_eph)) && gps_ephemeris_ != nullptr)
    {
        gps_ephemeris_(gps_eph);
    }
    ephemeris::GalileoEphemeris gal_eph;
    if (gnss_.sampleGalEph(t(), make_out(gal_eph)) && gal_ephemeris_ != nullptr)
    {
        gal_ephemeris_(gal_eph);
    }
    ephemeris::GlonassEphemeris glo_eph;
    if (gnss_.sampleGloEph(t(), make_out(glo_eph)) && glo_ephemeris_ != nullptr)
    {
        glo_ephemeris_(glo_eph);
    }
}

}  // namespace sim
}  // namespace mc
