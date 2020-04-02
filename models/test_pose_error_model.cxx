#include <gtest/gtest.h>

#include "common/numerical_jacobian.h"
#include "common/print.h"
#include "common/test_helpers.h"
#include "models/pose_error_model.h"

namespace mc {
namespace models {

using DQ = math::DQuat<double>;

TEST(PoseError, HandCrafted)
{
    const Vec6 Xi = Vec6::Ones();

    const Vec6 perturbation = 0.3 * Vec6::Random();

    const DQ anchor = DQ::Random();
    const DQ pose = anchor * DQ::exp(perturbation);

    PoseError f(anchor, Xi);

    const double* parameters[] = {pose.data()};

    Vec6 residuals;

    EXPECT_TRUE(f.Evaluate(parameters, residuals.data(), nullptr));

    MAT_EQ(residuals, perturbation);
}

TEST(PoseError, Jacobian)
{
    const Vec6 Xi = Vec6::Ones();

    const Vec6 perturbation = 0.3 * Vec6::Random();

    const DQ anchor = DQ::Random();
    const DQ pose = anchor * DQ::exp(perturbation);

    PoseError f(anchor, Xi);

    const auto fun = [&](const Vec8& _arr) -> Vec6 {
        const double* parameters[] = {_arr.data()};
        Vec6 residuals;
        f.Evaluate(parameters, residuals.data(), 0);
        return residuals;
    };

    const double* parameters[] = {pose.data()};
    MatRM68 jac;
    double* jacobians[] = {jac.data()};
    Vec6 residuals;
    EXPECT_TRUE(f.Evaluate(parameters, residuals.data(), jacobians));

    MatRM68 numerical_jac = compute_jac(Vec8(pose.arr_), fun);

    MAT_EQ(numerical_jac * pose.dParamDGen(), jac * pose.dParamDGen());
    MAT_EQ(residuals, perturbation);
}

}  // namespace models
}  // namespace mc
