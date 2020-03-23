#include "models/pose_error_model.h"

#include "common/matrix_defs.h"
#include "common/print.h"

namespace mc {
namespace models {

using DQ = math::DQuat<double>;

PoseError::PoseError(const math::DQuat<double>& anchor, const Vec6& Xi)
    : anchor_inv_(anchor.inverse()), Xi_(Xi)
{
}

bool PoseError::Evaluate(const double* const* parameters,
                         double* residuals,
                         double** jacobians) const
{
    const DQ pose(parameters[0]);

    Eigen::Map<Vec6> res(residuals);

    if (jacobians)
    {
        if (jacobians[0])
        {
            Mat6 dlog;
            res = Xi_.cwiseProduct((anchor_inv_ * pose).log<JacobianSide::RIGHT>(&dlog));

            Eigen::Map<MatRM68> jac(jacobians[0]);
            jac = (Xi_.asDiagonal() * dlog) * pose.dGenDParam();
            return true;
        }
    }
    res = Xi_.cwiseProduct((anchor_inv_ * pose).log());
    return true;
}
}  // namespace models
}  // namespace mc
