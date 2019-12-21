#include <random>

#include <gtest/gtest.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/geometry/so3.h"
#include "common/matrix_defs.h"
#include "common/test_helpers.h"

static constexpr int NUM_ITERS = 3;

TEST(SO3, rotation_direction)
{
    // Compare against a known active and passive rotation

    const Vec3 beta(1, 0, 0);
    const SO3<double> R_x_45 = SO3<double>::from_axis_angle(beta, 45.0 * M_PI / 180.0);
    Mat3 R_true;
    // clang-format off
    R_true << 1.0000000, 0.0000000,            0.0000000,
              0.0000000, 0.70710678118654757,  0.70710678118654757,
              0.0000000, -0.70710678118654757, 0.70710678118654757;
    // clang-format on
    MATRIX_EQUALS(R_x_45.matrix(), R_true);

    const Vec3 v(0, 0, 1);
    const Vec3 v_active_rotated(0, -1.0 * std::pow(0.5, 0.5), std::pow(0.5, 0.5));
    const Vec3 v_x_45 = R_x_45.rota(v);

    MATRIX_EQUALS(v_x_45, v_active_rotated);
    MATRIX_EQUALS(R_x_45.transpose() * v, v_active_rotated);
    MATRIX_EQUALS(R_true.transpose() * v, v_active_rotated);
    MATRIX_EQUALS(v_x_45, v_active_rotated);

    const Vec3 v_passive_rotated(0, std::pow(0.5, 0.5), std::pow(0.5, 0.5));
    const Vec3 v_x_45_T = R_x_45.rotp(v);

    MATRIX_EQUALS(v_x_45_T, v_passive_rotated);
    MATRIX_EQUALS(R_x_45 * v, v_passive_rotated);
    MATRIX_EQUALS(R_true * v, v_passive_rotated);
}

TEST(SO3, from_two_unit_vectors)
{
    Vec3 v1, v2;
    for (int i = 0; i < NUM_ITERS; i++)
    {
        // Check singularity condition
        if (i < NUM_ITERS / 3)
        {
            v1.setRandom();
            v2 = -v1 + Vec3::Random() * 1e-4;
        }
        else
        {
            v1.setRandom();
            v2.setRandom();
        }
        v1 /= v1.norm();
        v2 /= v2.norm();
        const SO3<double> R = SO3<double>::from_two_unit_vectors(v1, v2);
        MATRIX_EQUALS(R.rotp(v1), v2);
        MATRIX_EQUALS(R.rota(v2), v1);
    }
}

TEST(SO3, exp)
{
    // Check that qexp is right by comparing with matrix exp and axis-angle
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Vec3 omega;
        omega.setRandom();
        if (i < NUM_ITERS / 3)
        {
            // omega close to zero
            omega *= 1e-8;
        }
        else if (i < 2 * NUM_ITERS / 3)
        {
            // omega close to +/-pi
            omega *= pow(-1, i) * 1e-6 + M_PI / omega.norm();
        }
        const Mat3 R_omega_exp = skew(omega).exp();
        const SO3<double> R_exp(R_omega_exp);
        const SO3<double> R_ours = SO3<double>::exp(omega);
        SO3_EQUALS(R_exp, R_ours);
    }
}

TEST(SO3, exp_log_inverses)
{
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
        const Mat3 R_omega_exp = skew(omega).exp();
        const SO3<double> R_exp(R_omega_exp);
        const SO3<double> R_ours = SO3<double>::exp(omega);

        MATRIX_CLOSE_AMG_SIGN((SO3<double>::exp(omega)).log(), omega, 1e-7);
        EXPECT_NEAR((SO3<double>::exp(omega)).log().norm(), omega.norm(), 1e-7);
        SO3_CLOSE(SO3<double>::exp(R_ours.log()), R_ours, 1e-7);
    }
}

TEST(SO3, random)
{
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        const SO3<double> random = SO3<double>::Random();
        EXPECT_NEAR(random.matrix().determinant(), 1.0, 1e-8);
    }
}

TEST(SO3, orthonormalize)
{
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        SO3<double> R = SO3<double>::Random();
        const int row = rand() % 4;
        const int col = rand() % 4;
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

static SO3<double> operator+(const SO3<double>& R, const Vec3& delta)
{
    return R * SO3<double>::exp(delta);
}

static Vec3 operator-(const SO3<double>& R1, const SO3<double>& R2)
{
    return (R2.inverse() * R1).log();
}

TEST(SO3, boxplus_rules)
{
    const Vec3 zeros = Vec3::Zero();
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const SO3<double> R1 = SO3<double>::Random();
        const SO3<double> R2 = SO3<double>::Random();
        const Vec3 delta1 = Vec3::Random();
        const Vec3 delta2 = Vec3::Random();

        SO3_EQUALS(R1 + zeros, R1);
        SO3_EQUALS(R1 + (R2 - R1), R2);
        MATRIX_EQUALS((R1 + delta1) - R1, delta1);
        EXPECT_LE(((R1 + delta1) - (R1 + delta2)).norm(), (delta1 - delta2).norm());
    }
}

TEST(SO3, Adjoint)
{
    for (int j = 0; j < NUM_ITERS; ++j)
    {
        constexpr double eps = 1e-8;
        const SO3<double> R = SO3<double>::Random();
        Mat3 fd = Mat3::Zero();
        for (int i = 0; i < 3; ++i)
        {
            fd.col(i) = (R * SO3<double>::exp(Vec3::Unit(i) * eps) * R.inverse()).log() / eps;
        }

        MATRIX_CLOSE(fd, R.Ad(), 1e-3);
    }
}

TEST(SO3, AdjointIdentities)
{
    constexpr double eps = 1e-8;
    const SO3<double> R = SO3<double>::Random();
    const SO3<double> R2 = SO3<double>::Random();
    const Vec3 v = Vec3::Random();

    SO3_EQUALS(R * SO3<double>::exp(v), SO3<double>::exp(R.Ad() * v) * R);
    MATRIX_EQUALS((R * R2).Ad(), R.Ad() * R2.Ad());
}
