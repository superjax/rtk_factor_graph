#pragma once

#include <Eigen/Core>

#include "common/error.h"
#include "common/math/dquat.h"
#include "common/math/jet.h"
#include "common/matrix_defs.h"
#include "common/measurements/imu.h"
#include "common/out.h"
#include "common/utctime.h"
#include "factors/imu_state.h"

namespace mc {
namespace factors {

class ImuIntegrator
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    ImuIntegrator();
    void reset(const UTCTime& t);
    Error computeEndJet(const math::Jet<double>& start, Out<math::Jet<double>> end) const;
    Error integrate(const meas::ImuSample& imu);
    void dynamics(const ImuState& state,
                  const meas::ImuSample& imu_sample,
                  Out<ImuErrorState> dstate) const;

    inline const UTCTime& t0() const { return t0_; }
    inline const UTCTime& tf() const { return tf_; }
    inline const ImuState& state() const { return state_; }
    inline ImuState& state() { return state_; }
    inline double dt() const { return (tf_ - t0_).toSec(); }

    auto accelBias() { return bias_.head<3>(); }
    const auto accelBias() const { return bias_.head<3>(); }
    auto gyroBias() { return bias_.tail<3>(); }
    const auto gyroBias() const { return bias_.tail<3>(); }

 protected:
    Vec6 bias_;   // Imu Bias
    UTCTime t0_;  // start of interval
    UTCTime tf_;  // end of interval
    ImuState state_;
    meas::ImuSample prev_imu_;
};

}  // namespace factors
}  // namespace mc
