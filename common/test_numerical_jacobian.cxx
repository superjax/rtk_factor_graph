#include <gtest/gtest.h>

#include "common/defs.h"
#include "common/math/dquat.h"
#include "common/numerical_jacobian.h"
#include "common/test_helpers.h"

namespace mc {

TEST(NumericalJacobianHelper, Dof)
{
    EXPECT_EQ(dof<math::DQuat<double>>(), 6);
    EXPECT_EQ(dof<math::Quat<double>>(), 3);

    EXPECT_EQ(dof<Vec10>(), 10);

    // Doesn't Compile
    // EXPECT_EQ(dof<Mat10>(), 10);
    // EXPECT_EQ(dof<Eigen::MatrixXd>(), 10);
    // EXPECT_EQ(dof<Eigen::VectorXd>(), 10);
}

class OtherThing
{
};

class Thing
{
 public:
    Thing operator+(const OtherThing& x) { return Thing(); }
    OtherThing operator-(const Thing& x) { return OtherThing(); }
};

TEST(NumericalJacobianHelper, HasPlus)
{
    EXPECT_TRUE((has_plus<Thing, OtherThing>::value));
    EXPECT_FALSE((has_plus<OtherThing, OtherThing>::value));
    EXPECT_TRUE((has_plus<Vec3, Vec3>::value));
    EXPECT_TRUE((has_plus<Mat4, Mat4>::value));
    EXPECT_FALSE((has_plus<math::DQuat<double>, Vec6>::value));
}

TEST(NumericalJacobianHelper, HasMinus)
{
    EXPECT_TRUE((has_minus<Thing>::value));
    EXPECT_FALSE((has_minus<OtherThing>::value));
    EXPECT_TRUE((has_minus<Vec3>::value));
    EXPECT_TRUE((has_minus<Mat4>::value));
    EXPECT_FALSE((has_minus<math::DQuat<double>>::value));
}

TEST(NumericalJacobianHelper, perturb)
{
    MAT_EQ((perturb<JacobianSide::LEFT>(Vec3::Zero(), Vec3::Unit(0))), Vec3(1, 0, 0));
    MAT_EQ((perturb<JacobianSide::RIGHT>(Vec3::Zero(), Vec3::Unit(0))), Vec3(1, 0, 0));

    const math::Quatd q = math::Quatd::Random();
    const Vec3 v = Vec3::Random();

    QUAT_EQ(perturb<JacobianSide::LEFT>(q, v), math::Quatd::exp(v) * q);
    QUAT_EQ(perturb<JacobianSide::RIGHT>(q, v), q * math::Quatd::exp(v));
}

TEST(NumericalJacobianHelper, difference)
{
    MAT_EQ((difference<JacobianSide::LEFT>(Vec3::Zero(), Vec3::Unit(0))), Vec3(-1, 0, 0));
    MAT_EQ((difference<JacobianSide::RIGHT>(Vec3::Zero(), Vec3::Unit(0))), Vec3(-1, 0, 0));

    const math::Quatd q1 = math::Quatd::Random();
    const math::Quatd q2 = math::Quatd::Random();

    EXPECT_TRUE(detail::is_lie_group<math::Quatd>::value);

    MAT_EQ(Vec3(difference<JacobianSide::LEFT>(q2, q1)), ((q2 * q1.inverse()).log()));
    MAT_EQ(Vec3(difference<JacobianSide::RIGHT>(q2, q1)), ((q1.inverse() * q2).log()));
}

TEST(NumericalJacobianHelper, VectorFunction)
{
    const Mat4 jac = Mat4::Random();
    const auto fun = [&](const Vec4& x) { return jac * x; };

    const Vec4 x0 = Vec4::Random();

    const Mat4 fd = compute_jac(x0, fun);

    MATRIX_CLOSE(fd, jac, 1e-6);
}

TEST(NumericalJacobianHelper, LeftJacobian)
{
    const Vec3 v = Vec3::Random();
    const auto fun = [&](const math::Quatd& x) { return x.rotp(v); };

    const math::Quatd x0 = math::Quatd::Random();

    const Mat3 fd = compute_jac<JacobianSide::LEFT>(x0, fun);

    MATRIX_CLOSE(fd, x0.Ad().inverse() * skew(v), 1e-6);
}

TEST(NumericalJacobianHelper, PerturbJacobian)
{
    const Vec3 v = Vec3::Random();
    const auto fun = [&](const math::Quatd& x) { return x.rotp(v); };

    const math::Quatd x0 = math::Quatd::Random();

    const Mat3 fd = compute_jac<JacobianSide::RIGHT>(x0, fun);

    MATRIX_CLOSE(fd, skew(x0.Ad().inverse() * v), 1e-6);
}

TEST(NumericalJacobianHelper, DifferenceJacobian)
{
    Vec3 v = Vec3::Random();

    const auto fun = [](const Vec3& _v) { return math::Quatd::exp(_v); };
    const Mat3 left_fd = compute_jac<JacobianSide::LEFT>(v, fun);
    const Mat3 right_fd = compute_jac<JacobianSide::RIGHT>(v, fun);

    Mat3 left_jac_exp;
    math::Quatd::exp<JacobianSide::LEFT>(v, &left_jac_exp);
    Mat3 right_jac_exp;
    math::Quatd::exp<JacobianSide::RIGHT>(v, &right_jac_exp);

    MATRIX_CLOSE(left_jac_exp, left_fd, 1e-6);
    MATRIX_CLOSE(right_jac_exp, right_fd, 1e-6);
}

TEST(NumericalJacobianHelper, FullGroupJacobian)
{
    const math::Quatd q1 = math::Quatd::Random();
    const math::Quatd x0 = math::Quatd::Random();

    const auto fun = [&](const math::Quatd& x) { return q1 * x; };
    const Mat3 left_fd = compute_jac<JacobianSide::LEFT>(x0, fun);
    const Mat3 right_fd = compute_jac<JacobianSide::RIGHT>(x0, fun);

    MATRIX_CLOSE(left_fd, q1.Ad(), 1e-6);
    MATRIX_CLOSE(right_fd, Mat3::Identity(), 1e-6);
}

}  // namespace mc
