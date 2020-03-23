#include "models/imu_integrator.h"
#include "common/check.h"
#include "common/defs.h"
#include "common/matrix_defs.h"

namespace mc {
namespace models {

ImuIntegrator::ImuIntegrator()
{
    bias_.setZero();
    state_.setIdentity();
    t0_ = INVALID_TIME;
    tf_ = INVALID_TIME;
}

void ImuIntegrator::reset(const UTCTime& t)
{
    t0_ = t;
    tf_ = t;
    state_.setIdentity();
}

Error ImuIntegrator::computeEndJet(const math::Jet<double>& start, Out<math::Jet<double>> end) const
{
    check(std::abs(1.0 - state_.gamma.arr_.norm()) < 1e-8, "Itegrated rotation left manifold.");
    check(std::abs(1.0 - start.x.real().norm()) < 1e-8, "start pose rotation left manifold.");
    check(tf_ >= t0_, "Imu integration time going backwards.");

    const auto& pose_i = start.x;
    const auto& vel_i = start.linear_vel;

    if (tf_ == t0_)
    {
        *end = start;
        return Error::create("Zero integration time");
    }

    const double dt = (tf_ - t0_).toSec();

    // TODO: this could probably be significantly optimized
    const Vec3 translation_j = pose_i.translation() +
                               pose_i.rotation().rota(vel_i * dt + state_.alpha) +
                               0.5 * GRAVITY * dt * dt;
    const math::Quat<double> rot_j = pose_i.rotation() * state_.gamma;
    end->x = math::DQuat<double>(rot_j, translation_j);
    end->linear_vel = state_.gamma.rotp(vel_i + pose_i.rotation().rotp(GRAVITY) * dt + state_.beta);
    end->angular_vel = prev_imu_.gyro - gyroBias();

    return Error::none();
}

void ImuIntegrator::dynamics(const ImuState& state,
                             const meas::ImuSample& imu,
                             Out<ImuErrorState> dstate) const
{
    const Vec3 acceleration = imu.accel - accelBias();
    const Vec3 angular_rate = imu.gyro - gyroBias();

    dstate->alpha = state.beta;
    dstate->beta = state.gamma.rota(acceleration);
    dstate->gamma = angular_rate;
}

Error ImuIntegrator::integrate(const meas::ImuSample& imu)
{
    const double dt = (imu.t - tf_).toSec();
    if (dt <= 0)
        return Error::create("Cannot Integrate ImuIntegrator with dt <= 0");
    tf_ = imu.t;

    ImuErrorState dstate;
    dynamics(state_, imu, Out(dstate));
    state_ += (dstate * dt);

    prev_imu_ = imu;

    return Error::none();
}

}  // namespace models
}  // namespace mc
