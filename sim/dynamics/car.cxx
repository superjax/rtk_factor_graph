#include "sim/dynamics/car.h"

#include "common/print.h"

namespace mc {
namespace sim {
namespace dynamics {

SimCar::SimCar(const UTCTime& t, const SimCar::Options& options)
    : Dynamics(t, options),
      options_(options),
      heading_pid_(options.heading_controller_gains),
      velocity_pid_(options.velocity_controller_gains)
{
}

void SimCar::compute_wrench(const math::TwoJet<double>& state,
                            const Eigen::VectorXd& u,
                            Out<typename math::DQuat<double>::TangentVector> wrench) const
{
    const Vec3 Fl = e_x * u(0) / options_.wheel_radius;
    const Vec3 Fr = e_x * u(1) / options_.wheel_radius;
    const Vec3 r_l(-options_.track, options_.axle_width / 2.0, 0);
    const Vec3 r_r(-options_.track, -options_.axle_width / 2.0, 0);

    wrench->linear() = Fl + Fr;
    wrench->angular() = r_l.cross(Fl) + r_r.cross(Fr);
}

Vec2 SimCar::controller(const UTCTime& t, const Vec2& vw)
{
    const double dt = (t - this->prev_t_).toSec();
    const double v = vw(0);
    const double omega = vw(1);

    const double alp = heading_pid_.run(dt, this->x.dx.angular()(2), omega, false);
    const double acc = velocity_pid_.run(dt, this->x.dx.linear()(0), v, false);

    // Invert model
    const double F_l = 0.5 * (acc - alp / options_.axle_width);
    const double F_r = 0.5 * (alp / options_.axle_width + acc);

    const Vec2 u(F_l * options_.wheel_radius, F_r * options_.wheel_radius);
    return u;
}

SimCar::WaypointFollower::WaypointFollower(const SimCar::WaypointFollower::Options& options)
    : options_(options)
{
}

Vec2 SimCar::WaypointFollower::follow(const math::DQuat<double>& current_state)
{
    const Vec3 desired_position = options_.waypoints[current_waypoint_ % options_.waypoints.size()];
    const Vec3 position = current_state.translation();
    Vec3 error = desired_position - position;

    if (error.norm() < options_.close_enough_threshold)
    {
        ++current_waypoint_;
        error = options_.waypoints[current_waypoint_ % options_.waypoints.size()] - position;
    }
    const Vec3 error_direction = error.normalized();
    const Vec3 forward_direction = current_state.rota(e_x);

    const double d = error_direction.dot(forward_direction);
    const double xi = error_direction.cross(forward_direction).z();

    double vel;
    double w;
    if (abs(xi) > std::asin(deg2Rad(15)))
    {
        vel = 1.0;
        w = -options_.max_omega * xi;
    }
    else
    {
        vel = options_.max_speed * std::pow(d, 8);
        w = -options_.max_omega * xi;
    }

    return Vec2(vel, w);
}

}  // namespace dynamics
}  // namespace sim
}  // namespace mc
