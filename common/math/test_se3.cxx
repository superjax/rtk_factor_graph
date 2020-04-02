#include <gtest/gtest.h>

#include <random>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/math/se3.h"
#include "common/matrix_defs.h"
#include "common/numerical_jacobian.h"
#include "common/test_helpers.h"

namespace mc {
namespace math {

static constexpr int NUM_ITERS = 100;

Mat4 up(const Vec6& u)
{
    Mat4 mat;
    // clang-format off
        mat << 0, -u(2), u(1), u(3),
               u(2), 0, -u(0), u(4),
              -u(1), u(0), 0,  u(5),
              0,     0,    0,  0;
    // clang-format on
    return mat;
};

TEST(SE3, known_passive_transform)
{
    const SO3<double> R_a2b = SO3<double>::from_axis_angle(Vec3(0, 0, 1), M_PI / 4.0);
    const Vec3 t_a(1, 1, 0);
    const Vec3 t_b = R_a2b * -t_a;
    const Vec3 p_a(1, 0, 0);
    const SE3<double> T_a2b(R_a2b, t_b);
    const Vec3 p_b(-sqrt(0.5), -sqrt(0.5), 0);
    MAT_EQ(p_b, T_a2b * p_a);
    MAT_EQ(p_b, T_a2b.transformp(p_a));
}

TEST(SE3, known_active_transform)
{
    const SO3<double> R_a2b = SO3<double>::from_axis_angle(Vec3(0, 0, 1), M_PI / 4.0);
    const Vec3 t_a(1, 1, 0);
    const Vec3 t_b = R_a2b * -t_a;
    const Vec3 p_a(1, 0, 0);
    const SE3<double> T_a2b(R_a2b, t_b);
    const Vec3 p_b(1 + sqrt(0.5), 1 + sqrt(0.5), 0);
    MAT_EQ(p_b, T_a2b.transforma(p_a));
    MAT_EQ(p_b, T_a2b.inverse() * p_a);
}

TEST(SE3, inverse)
{
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        const SE3<double> T1 = SE3<double>::Random();
        const SE3<double> T2 = T1.inverse();
        const SE3<double> T3 = T1 * T2;
        SE3_EQUALS(T3, SE3<double>::identity());
    }
}

TEST(SE3, transform_vector)
{
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        const SE3<double> T1 = SE3<double>::Random();
        const Vec3 p = Vec3::Random();
        MAT_EQ(T1.transformp(T1.inverse().transformp(p)), p);
        MAT_EQ(T1.inverse().transformp(T1.transformp(p)), p);
        MAT_EQ(T1.transforma(T1.inverse().transforma(p)), p);
        MAT_EQ(T1.inverse().transforma(T1.transforma(p)), p);
    }
}

TEST(SE3, exp)
{
    // Check that qexp is right by comparing with matrix exp and axis-angle
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const Vec6 omega = Vec6::Random();
        const Mat4 T_omega_exp = up(omega).exp();
        SE3<double> T_exp(T_omega_exp);
        SE3<double> T_ours = SE3<double>::exp(omega);
        SE3_EQUALS(T_exp, T_ours);
    }
}

TEST(SE3, exp_log_inverses)
{
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        Vec6 wu = Vec6::Random();
        if (i < NUM_ITERS / 3.)
        {
            // Very small angle
            wu.head<3>() *= 1e-5;
        }

        const Mat4 T_wu_exp = up(wu).exp();
        const SE3<double> T_exp(T_wu_exp);
        const SE3<double> T_ours = SE3<double>::exp(wu);

        MATRIX_CLOSE_AMG_SIGN((SE3<double>::exp(wu)).log(), wu, 1e-7);
        EXPECT_NEAR((SE3<double>::exp(wu)).log().norm(), wu.norm(), 1e-7);
        SE3_CLOSE(SE3<double>::exp(T_ours.log()), T_ours, 1e-7);
    }
}

static SE3<double> operator+(const SE3<double>& T, const Vec6& v)
{
    return SE3<double>::exp(v) * T;
}
static Vec6 operator-(const SE3<double>& T1, const SE3<double>& T2)
{
    return (T1 * T2.inverse()).log();
}

TEST(SE3, boxplus_rules)
{
    Vec6 delta1, delta2, zeros;
    zeros.setZero();
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const SE3<double> T1 = SE3<double>::Random();
        const SE3<double> T2 = SE3<double>::Random();
        delta1.setRandom();
        delta2.setRandom();

        SE3_EQUALS(T1 + zeros, T1);
        SE3_EQUALS(T1 + (T2 - T1), T2);
        MAT_EQ((T1 + delta1) - T1, delta1);
        // Ethan and Adam say this rule is bogus for SE3
        // EXPECT_LE(((T1 + delta1) - (T1 + delta2)).norm(), (delta1 - delta2).norm());
    }
}

TEST(SE3, Adjoint)
{
    for (int j = 0; j < NUM_ITERS; ++j)
    {
        constexpr double eps = 1e-8;
        const SE3<double> T = SE3<double>::Random();
        Mat6 fd = Mat6::Zero();
        for (int i = 0; i < 6; ++i)
        {
            fd.col(i) = (T * SE3<double>::exp(Vec6::Unit(i) * eps) * T.inverse()).log() / eps;
        }

        MATRIX_CLOSE(fd, T.Ad(), 1e-3);
    }
}

TEST(SE3, AdjointIdentities)
{
    const SE3<double> T = SE3<double>::Random();
    const SE3<double> T2 = SE3<double>::Random();
    const Vec6 v = Vec6::Random();

    SE3_EQUALS(T * SE3<double>::exp(v), SE3<double>::exp(T.Ad() * v) * T);
    MAT_EQ((T * T2).Ad(), T.Ad() * T2.Ad());
}

TEST(SE3, GroupJacobians)
{
    const auto log_fun1 = [](const SE3<double>& x, const SE3<double>& y) { return (y * x); };
    const auto log_fun2 = [](const SE3<double>& x, const SE3<double>& y) {
        return (x.inverse() * y);
    };
    const auto log_fun3 = [](const SE3<double>& x, const SE3<double>& y) {
        return (y * x.inverse());
    };
    const auto log_fun4 = [](const SE3<double>& x, const SE3<double>& y) { return (x * y); };

    for (int i = 0; i < NUM_ITERS; ++i)
    {
        SE3<double> x = SE3<double>::Random();
        SE3<double> y = SE3<double>::Random();

        Mat6 right_jac1 = right_jac(x, log_fun1, y);
        Mat6 right_jac2 = right_jac(x, log_fun2, y);
        Mat6 right_jac3 = right_jac(x, log_fun3, y);
        Mat6 right_jac4 = right_jac(x, log_fun4, y);

        MATRIX_CLOSE(right_jac1, Mat6::Identity(), 1e-3);
        MATRIX_CLOSE(right_jac2, -(x.inverse() * y).Ad().inverse(), 1e-3);
        MATRIX_CLOSE(right_jac3, -x.Ad(), 1e-3);
        MATRIX_CLOSE(right_jac4, y.Ad().inverse(), 1e-3);

        Mat6 left_jac1 = left_jac(x, log_fun1, y);
        Mat6 left_jac2 = left_jac(x, log_fun2, y);
        Mat6 left_jac3 = left_jac(x, log_fun3, y);
        Mat6 left_jac4 = left_jac(x, log_fun4, y);

        MATRIX_CLOSE(left_jac1, y.Ad(), 1e-3);
        MATRIX_CLOSE(left_jac2, -x.Ad().inverse(), 1e-3);
        MATRIX_CLOSE(left_jac3, -(y * x.inverse()).Ad(), 1e-3);
        MATRIX_CLOSE(left_jac4, Mat6::Identity(), 1e-3);
    }
}

TEST(SE3, VectorJacobians)
{
    const auto passive = [](const SE3<double>& T, const Vec3& v) { return T.transformp(v); };
    const auto active = [](const SE3<double>& T, const Vec3& v) { return T.transforma(v); };

    typedef Eigen::Matrix<double, 3, 6> Mat36;
    const SE3<double> T = SE3<double>::Random();
    const Vec3 v = Vec3::Random();

    Mat36 right1 = right_jac2(T, passive, v);
    Mat36 right2 = right_jac2(T, active, v);

    MATRIX_CLOSE(right1, hstack(-T.R().Ad() * skew(v), T.R().Ad()), 1e-3);
    MATRIX_CLOSE(right2, hstack(skew(T.R().Ad().inverse() * (v - T.t())), -Mat3::Identity()), 1e-3);

    Mat36 left1 = left_jac2(T, passive, v);
    Mat36 left2 = left_jac2(T, active, v);

    MATRIX_CLOSE(left1, hstack(-skew(T.R() * v + T.t()), Mat3::Identity()), 1e-3);
    MATRIX_CLOSE(left2, hstack(T.R().inverse().Ad() * skew(v), -T.R().inverse().Ad()), 1e-3);
}

TEST(SE3, ExpLogJacobians)
{
    const auto exp = [](const Vec6& v) { return SE3<double>::exp(v); };
    const auto log = [](const SE3<double>& R) { return R.log(); };

    const SE3<double> T = SE3<double>::Random();
    const Vec6 v = Vec6::Random();

    const Mat6 exp_left = left_jac3(v, exp);
    Mat6 jac_exp;
    SE3<double>::exp(v, &jac_exp);

    MATRIX_CLOSE(jac_exp, exp_left, 1e-6);

    const Mat6 log_left = left_jac2(T, log);
    Mat6 jac_log;
    T.log(&jac_log);

    MATRIX_CLOSE(jac_log, log_left, 1e-6);

    const Mat6 exp_right = right_jac3(v, exp);
    jac_exp.topLeftCorner<3, 3>().transposeInPlace();
    jac_exp.bottomLeftCorner<3, 3>().transposeInPlace();
    jac_exp.bottomRightCorner<3, 3>().transposeInPlace();

    MATRIX_CLOSE(jac_exp, exp_right, 1e-6);

    Mat6 log_right = right_jac2(T, log);
    jac_log.topLeftCorner<3, 3>().transposeInPlace();
    jac_log.bottomLeftCorner<3, 3>().transposeInPlace();
    jac_log.bottomRightCorner<3, 3>().transposeInPlace();

    MATRIX_CLOSE(jac_log, log_right, 1e-6);
}

}  // namespace math
}  // namespace mc
