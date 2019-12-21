#include <random>

#include <gtest/gtest.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/geometry/se3.h"
#include "common/matrix_defs.h"
#include "common/test_helpers.h"

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
    MATRIX_EQUALS(p_b, T_a2b * p_a);
    MATRIX_EQUALS(p_b, T_a2b.transformp(p_a));
}

TEST(SE3, known_active_transform)
{
    const SO3<double> R_a2b = SO3<double>::from_axis_angle(Vec3(0, 0, 1), M_PI / 4.0);
    const Vec3 t_a(1, 1, 0);
    const Vec3 t_b = R_a2b * -t_a;
    const Vec3 p_a(1, 0, 0);
    const SE3<double> T_a2b(R_a2b, t_b);
    const Vec3 p_b(1 + sqrt(0.5), 1 + sqrt(0.5), 0);
    MATRIX_EQUALS(p_b, T_a2b.transforma(p_a));
    MATRIX_EQUALS(p_b, T_a2b.inverse() * p_a);
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
        MATRIX_EQUALS(T1.transformp(T1.inverse().transformp(p)), p);
        MATRIX_EQUALS(T1.inverse().transformp(T1.transformp(p)), p);
        MATRIX_EQUALS(T1.transforma(T1.inverse().transforma(p)), p);
        MATRIX_EQUALS(T1.inverse().transforma(T1.transforma(p)), p);
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
        MATRIX_EQUALS((T1 + delta1) - T1, delta1);
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
    MATRIX_EQUALS((T * T2).Ad(), T.Ad() * T2.Ad());
}
