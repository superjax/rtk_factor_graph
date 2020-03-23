#pragma once

#include "common/matrix_defs.h"

#include "common/utctime.h"

namespace mc {
namespace models {

class GnssDynamics
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    GnssDynamics(const UTCTime& t0, const UTCTime& tf, const Vec3& Xi);

    bool Evaluate(const double* const* parameters, double* residuals, double** jacobians) const;

 private:
    const double dt_;
    const Vec3& Xi_;  // diagonal of sqrt inverse covariance
};

}  // namespace models
}  // namespace mc
