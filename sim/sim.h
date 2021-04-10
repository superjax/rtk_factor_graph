#pragma once

#include <functional>

#include "common/measurements/gnss_observation.h"
#include "common/measurements/imu.h"
#include "common/random.h"
#include "sim/dynamics/car.h"
#include "sim/dynamics/multirotor.h"
#include "sim/sensors/gnss.h"
#include "sim/sensors/imu.h"

namespace mc {
namespace sim {

class Sim
{
 public:
    struct Options
    {
        double input_noise = 0.01;
        dynamics::SimCar::WaypointFollower::Options wp_options;
        dynamics::SimCar::Options car_options;
        sensors::GnssSim::Options gnss_options;
        sensors::ImuSim::Options imu_options;
    };

    Sim(const Options& options);

    void step(const double dt);

    inline void registerGnssCB(
        std::function<void(std::vector<meas::GnssObservation>)> gnss_callback)
    {
        gnss_callback_ = gnss_callback;
    }
    inline void registerImuCB(std::function<void(meas::ImuSample)> imu_callback)
    {
        imu_callback_ = imu_callback;
    }

    inline void registerGloEphCb(std::function<void(const mc::ephemeris::GlonassEphemeris&)> cb)
    {
        glo_ephemeris_ = cb;
    }
    inline void registerGalEphCb(std::function<void(const mc::ephemeris::GalileoEphemeris&)> cb)
    {
        gal_ephemeris_ = cb;
    }
    inline void registerGPSEphCb(std::function<void(const mc::ephemeris::GPSEphemeris&)> cb)
    {
        gps_ephemeris_ = cb;
    }

    inline const UTCTime& t() const { return dynamics_.t(); }
    inline const math::TwoJet<double>& x() const { return dynamics_.x; }
    inline const std::string& logPath() const { return gnss_.log_path_; }
    inline const math::DQuat<double>& T_e2g() const { return gnss_.T_e2g_; }
    inline const Vec2& clk() const { return gnss_.clock_bias_; }

 private:
    Options options_;
    dynamics::SimCar::WaypointFollower waypoint_follower_;
    dynamics::SimCar dynamics_;
    sensors::GnssSim gnss_;
    sensors::ImuSim imu_;

    std::function<void(std::vector<meas::GnssObservation>)> gnss_callback_;
    std::function<void(meas::ImuSample)> imu_callback_;

    std::function<void(const mc::ephemeris::GlonassEphemeris&)> glo_ephemeris_;
    std::function<void(const mc::ephemeris::GalileoEphemeris&)> gal_ephemeris_;
    std::function<void(const mc::ephemeris::GPSEphemeris&)> gps_ephemeris_;
};

}  // namespace sim
}  // namespace mc
