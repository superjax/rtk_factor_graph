#include "sim/dynamics/dynamics.h"

#include "common/print.h"

namespace mc {
namespace sim {
namespace dynamics {

Dynamics::Dynamics(const UTCTime& t, const Options& options) : options_(options), prev_t_(t)
{
    x = math::TwoJet<double>::Identity();
}

math::TwoJet<double> plus(const math::TwoJet<double>& x, const Vec6& d2x, const double dt)
{
    math::TwoJet<double> out;
    out.x = x.x * math::DQuat<double>::exp(x.dx * dt + x.d2x * dt * dt * 0.5);
    out.dx = x.dx + d2x * dt;
    out.d2x = d2x;
    return out;
}

void Dynamics::f(const math::TwoJet<double>& state,
                 const Eigen::VectorXd& u,
                 Out<typename math::DQuat<double>::TangentVector> d2x) const
{
    math::DQuat<double>::TangentVector wrench;
    this->compute_wrench(state, u, Out(wrench));
    d2x->linear() = wrench.linear() / options_.mass;
    d2x->angular() = (wrench.angular().array() / options_.inertia_.array()).matrix();
}

void Dynamics::step(const UTCTime& t, const Eigen::VectorXd& u)
{
    const double dt = (t - prev_t_).toSec();
    prev_t_ = t;

    math::DQuat<double>::TangentVector d2x;
    f(x, u, Out(d2x));
    x = plus(x, d2x, dt);

    // math::DQuat<double>::TangentVector k1, k2, k3, k4;

    // f(x, u, Out(k1));
    // f(plus(x, k1, dt / 2.0), u, Out(k2));
    // f(plus(x, k2, dt / 2.0), u, Out(k3));
    // f(plus(x, k3, dt), u, Out(k4));

    // Vec6 dx = (k1 + k2 * 2.0 + k3 * 2.0 + k4);
    // x = plus(x, dx, dt / 6.0);
}

}  // namespace dynamics
}  // namespace sim
}  // namespace mc
