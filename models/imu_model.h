#pragma once

#include "common/out.h"
#include "models/imu_integrator.h"

namespace mc {
namespace models {

class ImuModel : public ImuIntegrator
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    ImuModel(const UTCTime& t0, const Vec6& bias);

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

    Error integrate(const meas::ImuSample& u, const Mat6& cov);
    Error finished();

    bool Evaluate(const double* const* parameters, double* residual, double** jacobians) const;

    const Mat96& biasJacobian() const { return dstate_dbias_; }

    const Mat9& Xi() { return covariance_inv_sqrt_; }

 private:
    Mat6 prev_imu_covariance_;
    int num_updates_;

    Mat9 covariance_;
    Mat9 covariance_inv_sqrt_;

    Mat96 dstate_dbias_;
};

}  // namespace models
}  // namespace mc
