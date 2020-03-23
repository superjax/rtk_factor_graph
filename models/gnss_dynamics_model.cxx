#include "models/gnss_dynamics_model.h"

#include "common/print.h"

namespace mc {
namespace models {

GnssDynamics::GnssDynamics(const UTCTime& t0, const UTCTime& tf, const Vec3& Xi)
    : dt_((tf - t0).toSec()), Xi_(Xi)
{
}

template <int N>
struct VecJacobian final : public Eigen::Map<Eigen::Matrix<double, 3, N, Eigen::RowMajor>>
{
    VecJacobian(double* ptr) : Eigen::Map<Eigen::Matrix<double, 3, N, Eigen::RowMajor>>(ptr) {}
    auto clock() { return this->row(0); }
    auto rate() { return this->row(1); }
    auto sw() { return this->row(2); }
};

bool GnssDynamics::Evaluate(const double* const* parameters,
                            double* residuals,
                            double** jacobians) const
{
    const double& clock_bias1(parameters[0][0]);
    const double& clock_bias1_rate(parameters[0][1]);
    const double& clock_bias2(parameters[1][0]);
    const double& clock_bias2_rate(parameters[1][1]);
    const double& sw1(parameters[2][0]);
    const double& sw2(parameters[3][0]);

    const double avg_clock_bias_rate = (clock_bias1_rate + clock_bias2_rate) / 2.0;
    residuals[0] = Xi_[0] * (clock_bias2 - (clock_bias1 + avg_clock_bias_rate * dt_));
    residuals[1] = Xi_[1] * (clock_bias2_rate - clock_bias1_rate);
    residuals[2] = Xi_[2] * (sw2 - sw1);

    // clang-format off
    if (jacobians)
    {
        // dres_dclk1
        if (jacobians[0])
        {
            VecJacobian<2> jac(jacobians[0]);
            jac.clock()<< -Xi_[0], -Xi_[0] * dt_ / 2.0;
            jac.rate() << 0,       -Xi_[1];
            jac.sw().setZero();
        }

        // dres_dclk2
        if (jacobians[1])
        {
            VecJacobian<2> jac(jacobians[1]);
            jac.clock() << Xi_[0], -Xi_[0] * dt_ / 2.0;
            jac.rate()  << 0,       Xi_[1];
            jac.sw().setZero();
        }

        // dres_sw1
        if (jacobians[2])
        {
            jacobians[2][0] = 0;
            jacobians[2][1] = 0;
            jacobians[2][2] = -Xi_[2];
        }

        // dres_sw2
        if (jacobians[3])
        {
            jacobians[3][0] = 0;
            jacobians[3][1] = 0;
            jacobians[3][2] = Xi_[2];
        }
    }
    // clang-format on

    return true;
}

}  // namespace models
}  // namespace mc
