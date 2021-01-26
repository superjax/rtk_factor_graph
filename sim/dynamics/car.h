#pragma once

#include <memory>

#include "common/math/two_jet.h"
#include "common/out.h"
#include "sim/controllers/pid.h"
#include "sim/dynamics/dynamics.h"

namespace mc {
namespace sim {
namespace dynamics {

struct SimCar final : public Dynamics
{
    struct Options final : public Dynamics::Options
    {
        UTCTime t0;
        double wheel_radius = 0.1;
        double axle_width = 1.3;
        double track = 2.0;
        controllers::PID::Options heading_controller_gains;
        controllers::PID::Options velocity_controller_gains;
    };

    struct WaypointFollower
    {
        struct Options
        {
            std::vector<Vec3> waypoints = {{0, 0, 0},
                                           {100, 100, 0},
                                           {-100, 100, 0},
                                           {-100, -100, 0},
                                           {100, -100, 0}};
            double close_enough_threshold = 0.5;
            double max_speed = 10.0;
            double max_omega = 1.0;
        };

        WaypointFollower(const WaypointFollower::Options& options);
        Vec2 follow(const math::DQuat<double>& current_state);
        Vec3 current_waypoint() const
        {
            return options_.waypoints[current_waypoint_ % options_.waypoints.size()];
        }
        Options options_;
        int current_waypoint_;
    };

    SimCar(const Options& options);

    void compute_wrench(const math::TwoJet<double>& x,
                        const Eigen::VectorXd& u,
                        Out<typename math::DQuat<double>::TangentVector> wrench) const override;

    Vec2 controller(const UTCTime& t, const Vec2& vw);

 private:
    Options options_;
    controllers::PID heading_pid_;
    controllers::PID velocity_pid_;
};

}  // namespace dynamics
}  // namespace sim
}  // namespace mc
