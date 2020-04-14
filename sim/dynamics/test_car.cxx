#include <gtest/gtest.h>

#include "common/logger.h"
#include "common/random.h"
#include "sim/dynamics/car.h"

namespace mc {
namespace sim {
namespace dynamics {

TEST(SimCar, Random)
{
    SimCar::Options options;
    UTCTime t(0);

    SimCar car(t, options);

    Logger log("/tmp/mc/SimCar.Random.log");

    const double dt = 0.001;
    const double noise = 0.1;
    Vec2 u = Vec2::Constant(0.1);

    while (t < UTCTime(1.0))
    {
        t += dt;
        car.step(t, u);

        log.log(t.toSec(), car.x.x.translation(), car.x.x.rotation().log(), car.x.dx, car.x.d2x, u,
                Vec2::Zero(), Vec3::Constant(NAN));
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

    UTCTime t(0);

    SimCar car(t, options);

    Logger log("/tmp/mc/SimCar.Controller.log");

    const double dt = 0.001;
    const double noise = 0.01;
    Vec2 vw = Vec2(1, 0);

    while (t < UTCTime(1.0))
    {
        t += dt;
        const Vec2 u = car.controller(t, vw) + dt * randomNormal<Vec2>() * noise;
        car.step(t, u);

        log.log(t.toSec(), car.x.x.translation(), car.x.x.rotation().log(), car.x.dx, car.x.d2x, u,
                vw, Vec3::Constant(NAN));
    }
}

TEST(SimCar, Waypoints)
{
    SimCar::Options options;
    options.heading_controller_gains.kp = 10.0;
    options.heading_controller_gains.kd = 1.0;
    options.velocity_controller_gains.kp = 10.0;
    options.velocity_controller_gains.kd = 1.0;

    UTCTime t(0);

    SimCar car(t, options);

    SimCar::WaypointFollower::Options wp;
    wp.waypoints.push_back(Vec3(10, 10, 0));
    wp.waypoints.push_back(Vec3(20, -10, 0));
    wp.waypoints.push_back(Vec3(-40, 40, 0));
    wp.waypoints.push_back(Vec3(-00, -20, 0));
    SimCar::WaypointFollower follower(wp);

    Logger log("/tmp/mc/SimCar.Waypoints.log");

    const double dt = 0.001;
    const double noise = 0.01;

    while (t < UTCTime(60.0))
    {
        t += dt;
        const Vec2 vw = follower.follow(car.x.x);
        const Vec2 u = car.controller(t, vw) + dt * randomNormal<Vec2>() * noise;
        car.step(t, u);

        log.log(t.toSec(), car.x.x.translation(), car.x.x.rotation().log(), car.x.dx, car.x.d2x, u,
                vw, follower.current_waypoint());
    }
}

}  // namespace dynamics
}  // namespace sim
}  // namespace mc
