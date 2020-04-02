#include "core/estimator.h"

namespace mc {
namespace core {

Estimator::Estimator(const Estimator::Options& options)
    : options_(options), graph_(options.fg_options)
{
}

void Estimator::currentState(Out<UTCTime> t, Out<math::DQuat<double>> pose, Out<Vec3> vel) const {}

void Estimator::imuCb(const meas::ImuSample& imu) {}

}  // namespace core
}  // namespace mc
