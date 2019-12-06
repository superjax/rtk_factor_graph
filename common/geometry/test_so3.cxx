#include <random>

#include <gtest/gtest.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/geometry/so3.h"
#include "common/matrix_defs.h"
#include "common/test_helpers.h"

static constexpr int NUM_ITERS = 100;

TEST(SO3, exp)
{
    // Check that qexp is right by comparing with matrix exp and axis-angle
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Vec3 omega;
        omega.setRandom();
        Mat3 R_omega_exp = skew(omega).exp();
        SO3<double> R_exp(R_omega_exp);
        SO3<double> R_ours = SO3<double>::exp(omega);
        SO3_EQUALS(R_exp, R_ours);
    }
}

TEST(SO3, exp_log_inverses)
{
    // Check that qexp is right by comparing with matrix exp and axis-angle
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Vec3 omega;
        omega.setRandom();

        if (i < NUM_ITERS / 3.0)
        {
            // Very small omega
            omega *= 1e-5;
        }
        else if (i < 3. * NUM_ITERS / 6.)
        {
            // Really close to Pi (just below Pi)
            omega = (M_PI - 1e-8) * omega.normalized();
        }
        else if (i < 4. * NUM_ITERS / 6.)
        {
            // Really close to Pi (just above)
            omega = (M_PI + 1e-8) * omega.normalized();
        }
        Mat3 R_omega_exp = skew(omega).exp();
        SO3<double> R_exp(R_omega_exp);
        SO3<double> R_ours = SO3<double>::exp(omega);

        MATRIX_CLOSE_AMG_SIGN((SO3<double>::exp(omega)).log(), omega, 1e-7);
        EXPECT_NEAR((SO3<double>::exp(omega)).log().norm(), omega.norm(), 1e-7);
        SO3_CLOSE(SO3<double>::exp(R_ours.log()), R_ours, 1e-7);
    }
}

TEST(SO3, random)
{
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        SO3<double> random = SO3<double>::Random();
        EXPECT_NEAR(random.matrix().determinant(), 1.0, 1e-8);
    }
}

TEST(SO3, orthonormalize)
{
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        SO3<double> R = SO3<double>::Random();
        int row = rand() % 4;
        int col = rand() % 4;
        if (row != 3)
        {
            R.matrix().row(row) += 1e-3 * Vec3::Random();
        }
        if (col != 3)
        {
            R.matrix().col(col) += 1e-3 * Vec3::Random();
        }
        R.rectify();
        EXPECT_NEAR(R.matrix().determinant(), 1.0, 1e-8);
    }
}

TEST(SO3, boxplus_rules)
{
    Vec3 delta1, delta2, zeros;
    zeros.setZero();
    for (int i = 0; i < NUM_ITERS; i++)
    {
        SO3<double> R1 = SO3<double>::Random();
        SO3<double> R2 = SO3<double>::Random();
        delta1.setRandom();
        delta2.setRandom();

        SO3_EQUALS(R1 + zeros, R1);
        SO3_EQUALS(R1 + (R2 - R1), R2);
        MATRIX_EQUALS((R1 + delta1) - R1, delta1);
        EXPECT_LE(((R1 + delta1) - (R1 + delta2)).norm(), (delta1 - delta2).norm());
    }
}
