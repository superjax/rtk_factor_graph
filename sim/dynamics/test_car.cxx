#include <gtest/gtest.h>

#include "common/logging/logger.h"
#include "common/random.h"
#include "sim/dynamics/car.h"

namespace mc {
namespace sim {
namespace dynamics {

TEST(SimCar, Random)
{
    SimCar::Options options;
    options.t0 = UTCTime(0);

    SimCar car(options);

    logging::Logger log("/tmp/mc/SimCar.Random.log");

    const double dt = 0.001;
    const double noise = 0.1;
    Vec2 u = Vec2::Constant(0.1);

    while (car.t() < UTCTime(1.0))
    {
        car.step(car.t() + dt, u);

        log.log(car.t().toSec(), car.x.x.translation(), car.x.x.rotation().log(), car.x.dx,
                car.x.d2x, u, Vec2::Zero(), Vec3::Constant(NAN));
        u += dt * randomNormal<Vec2>() * noise;
    }
}

TEST(SimCar, Controller)
{
    SimCar::Options options;
    options.heading_controller_gains.kp = 10.0;
    options.heading_controller_gains.kd = 1.0;
    options.velocity_controller_gains.kp = 10.0;
    options.velocity_controller_gains.kd = 1.0;
    options.t0 = UTCTime(0);

    SimCar car(options);

    logging::Logger log("/tmp/mc/SimCar.Controller.log");

    const double dt = 0.001;
    const double noise = 0.01;
    Vec2 vw = Vec2(1, 0);

    while (car.t() < UTCTime(1.0))
    {
        const Vec2 u = car.controller(car.t() + dt, vw) + dt * randomNormal<Vec2>() * noise;
        car.step(car.t() + dt, u);

        log.log(car.t().toSec(), car.x.x.translation(), car.x.x.rotation().log(), car.x.dx,
                car.x.d2x, u, vw, Vec3::Constant(NAN));
    }
}

TEST(SimCar, Waypoints)
{
    SimCar::Options options;
    options.heading_controller_gains.kp = 10.0;
    options.heading_controller_gains.kd = 1.0;
    options.velocity_controller_gains.kp = 10.0;
    options.velocity_controller_gains.kd = 1.0;
    options.t0 = UTCTime(0);

    SimCar car(options);

    SimCar::WaypointFollower::Options wp;
    wp.waypoints.push_back(Vec3(10, 10, 0));
    wp.waypoints.push_back(Vec3(20, -10, 0));
    wp.waypoints.push_back(Vec3(-40, 40, 0));
    wp.waypoints.push_back(Vec3(-00, -20, 0));
    SimCar::WaypointFollower follower(wp);

    logging::Logger log("/tmp/mc/SimCar.Waypoints.log");

    const double dt = 0.001;
    const double noise = 0.01;

    while (car.t() < UTCTime(60.0))
    {
        const Vec2 vw = follower.follow(car.x.x);
        const Vec2 u = car.controller(car.t() + dt, vw) + dt * randomNormal<Vec2>() * noise;
        car.step(car.t() + dt, u);

        log.log(car.t().toSec(), car.x.x.translation(), car.x.x.rotation().log(), car.x.dx,
                car.x.d2x, u, vw, follower.current_waypoint());
    }
}

}  // namespace dynamics
}  // namespace sim
}  // namespace mc
