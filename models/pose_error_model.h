#pragma once

#include "common/math/dquat.h"
#include "common/matrix_defs.h"

namespace mc {
namespace models {

class PoseError
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PoseError(const math::DQuat<double>& anchor, const Vec6& Xi_);

    bool Evaluate(const double* const* parameters, double* residuals, double** jacobians) const;

 private:
    const math::DQuat<double> anchor_inv_;
    const Vec6 Xi_;
};

}  // namespace models
}  // namespace mc
