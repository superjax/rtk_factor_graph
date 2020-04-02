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

    inline const UTCTime& t() const { return dynamics_.t(); }
    inline const math::TwoJet<double>& x() const { return dynamics_.x; }

 private:
    Options options_;
    dynamics::SimCar::WaypointFollower waypoint_follower_;
    dynamics::SimCar dynamics_;
    sensors::GnssSim gnss_;
    sensors::ImuSim imu_;

    std::function<void(std::vector<meas::GnssObservation>)> gnss_callback_;
    std::function<void(meas::ImuSample)> imu_callback_;
};

}  // namespace sim
}  // namespace mc