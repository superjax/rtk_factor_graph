#pragma once
#include <Eigen/Core>

#include "common/error.h"
#include "common/math/dquat.h"
#include "common/matrix_defs.h"
#include "common/measurements/imu.h"
#include "common/out.h"
#include "common/utctime.h"
#include "core/states/pose.h"
#include "core/states/vel.h"
#include "models/imu_state.h"

namespace mc {
namespace models {

class ImuModel
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    ImuModel(const meas::ImuSample& z0, const Vec6& bias, const Vec6& R);

    Error computeEndState(const math::DQuat<double>& start,
                          const Vec3& start_vel,
                          Out<math::DQuat<double>> end,
                          Out<Vec3> end_vel) const;

    void errorStateDynamics(const ImuState& state,
                            const ImuErrorState& error_state,
                            const meas::ImuSample& imu,
                            const Vec6& input_noise,
                            Out<ImuErrorState> error_state_dot) const;

    // ydot = f(y, u) <-- nonlinear dynamics (reference state)
    // A = d(dydot)/d(dy) <-- error state
    // B = d(dydot)/d(eta) <-- error state
    // Because of the error state, ydot != Ay+Bu
    void dynamics(const ImuState& state,
                  const meas::ImuSample& imu,
                  Out<ImuErrorState> state_dot,
                  Out<Mat9> A,
                  Out<Mat96> B) const;

    Error integrate(const meas::ImuSample& u);
    Error finished();

    bool Evaluate(const double* const* parameters, double* residual, double** jacobians) const;

    const Mat9& Xi() { return covariance_inv_sqrt_; }

    // Splits this ImuModel into two.  (The called instances becomes the first interval, and the
    // returned object becomes the new instance)
    ImuModel split(const UTCTime& t);

    inline const UTCTime& t0() const { return history_.front().z.t; }
    inline const UTCTime& tf() const { return history_.back().z.t; }
    inline double delta_t() const { return (tf() - t0()).toSec(); }
    int numUpdates() const { return history_.size() - 1; }

    inline const ImuState& state() const { return history_.back().state; }
    inline ImuState& state() { return history_.back().state; }
    inline const Mat9& covariance() const { return history_.back().covariance; }
    inline Mat9& covariance() { return history_.back().covariance; }
    inline const Mat96& dStatedBias() const { return history_.back().dstate_dbias; }
    inline Mat96& dStatedBias() { return history_.back().dstate_dbias; }

    auto accelBias() { return bias_.head<3>(); }
    const auto accelBias() const { return bias_.head<3>(); }
    auto gyroBias() { return bias_.tail<3>(); }
    const auto gyroBias() const { return bias_.tail<3>(); }

 private:
    Vec6 bias_;      // Imu Bias
    const Vec6& R_;  // IMU covariance diagonal

    Mat9 covariance_inv_sqrt_;

    struct History
    {
        meas::ImuSample z;
        ImuState state;
        Mat9 covariance;
        Mat96 dstate_dbias;
    };

    std::vector<History> history_;
};

}  // namespace models
}  // namespace mc
