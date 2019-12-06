#include <random>

#include <gtest/gtest.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/geometry/se3.h"
#include "common/matrix_defs.h"
#include "common/test_helpers.h"

static constexpr int NUM_ITERS = 100;

Mat4 up(Vec6 u)
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

TEST(SE3, exp)
{
    // Check that qexp is right by comparing with matrix exp and axis-angle
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Vec6 omega;
        omega.setRandom();
        Mat4 T_omega_exp = up(omega).exp();
        SE3<double> T_exp(T_omega_exp);
        SE3<double> T_ours = SE3<double>::exp(omega);
        SE3_EQUALS(T_exp, T_ours);
    }
}

TEST(SE3, exp_log_inverses)
{
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        Vec6 wu;
        wu.setRandom();
        if (i < NUM_ITERS / 3.)
        {
            // Very small angle
            wu.head<3>() *= 1e-5;
        }

        Mat4 T_wu_exp = up(wu).exp();
        SE3<double> T_exp(T_wu_exp);
        SE3<double> T_ours = SE3<double>::exp(wu);

        MATRIX_CLOSE_AMG_SIGN((SE3<double>::exp(wu)).log(), wu, 1e-7);
        EXPECT_NEAR((SE3<double>::exp(wu)).log().norm(), wu.norm(), 1e-7);
        SE3_CLOSE(SE3<double>::exp(T_ours.log()), T_ours, 1e-7);
    }
}

TEST(SE3, boxplus_rules)
{
    Vec6 delta1, delta2, zeros;
    zeros.setZero();
    for (int i = 0; i < NUM_ITERS; i++)
    {
        SE3<double> T1 = SE3<double>::Random();
        SE3<double> T2 = SE3<double>::Random();
        delta1.setRandom();
        delta2.setRandom();

        SE3_EQUALS(T1 + zeros, T1);
        SE3_EQUALS(T1 + (T2 - T1), T2);
        MATRIX_EQUALS((T1 + delta1) - T1, delta1);
        // Ethan and Adam say this rule is bogus
        // EXPECT_LE(((T1 + delta1) - (T1 + delta2)).norm(), (delta1 - delta2).norm());
    }
}
