#include <random>

#include <gtest/gtest.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/geometry/so3.h"
#include "common/matrix_defs.h"
#include "common/numerical_jacobian.h"
#include "common/print.h"
#include "common/test_helpers.h"

static constexpr int NUM_ITERS = 24;

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

Vec3 get_new_sample()
{
    Vec3 omega = Vec3::Random();
    bool new_sample;
    static int balance = 0;
    do
    {
        new_sample = false;
        Mat3 R = SO3<double>::exp(omega);
        double R00 = R(0, 0);
        double R11 = R(1, 1);
        double R22 = R(2, 2);
        switch (balance)
        {
        case 0:
            new_sample = R00 > R11 && R00 > R22;
            break;
        case 1:
            new_sample = R11 > R00 && R11 > R22;
            break;
        case 2:
            new_sample = R22 > R00 && R22 > R11;
            break;
        }
        if (!new_sample)
        {
            omega.setRandom();
        }
    } while (!new_sample);
    balance = (balance + 1) % 3;
    return omega;
}

TEST(SO3, exp_log_inverses)
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Vec3 omega = get_new_sample();

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
            // Really close to Pi (just above Pi)
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
    const SO3<double> R = SO3<double>::Random();
    const SO3<double> R2 = SO3<double>::Random();
    const Vec3 v = Vec3::Random();

    SO3_EQUALS(R * SO3<double>::exp(v), SO3<double>::exp(R.Ad() * v) * R);
    MATRIX_EQUALS((R * R2).Ad(), R.Ad() * R2.Ad());
}

TEST(SO3, GroupJacobians)
{
    const auto log_fun1 = [](const SO3<double>& R1, const SO3<double>& R2) {
        return (R2.inverse() * R1);
    };
    const auto log_fun2 = [](const SO3<double>& R1, const SO3<double>& R2) {
        return (R1.inverse() * R2);
    };
    const auto log_fun3 = [](const SO3<double>& R1, const SO3<double>& R2) {
        return (R2 * R1.inverse());
    };
    const auto log_fun4 = [](const SO3<double>& R1, const SO3<double>& R2) {
        return (R1 * R2.inverse());
    };

    for (int i = 0; i < NUM_ITERS; ++i)
    {
        SO3<double> R1 = SO3<double>::Random();
        SO3<double> R2 = SO3<double>::Random();

        Mat3 right_jac1 = right_jac(R1, log_fun1, R2);
        Mat3 right_jac2 = right_jac(R1, log_fun2, R2);
        Mat3 right_jac3 = right_jac(R1, log_fun3, R2);
        Mat3 right_jac4 = right_jac(R1, log_fun4, R2);

        MATRIX_CLOSE(right_jac1, Mat3::Identity(), 1e-3);
        MATRIX_CLOSE(right_jac2, -(R2.inverse() * R1).Ad(), 1e-3);
        MATRIX_CLOSE(right_jac3, -R1.Ad(), 1e-3);
        MATRIX_CLOSE(right_jac4, R2.Ad(), 1e-3);

        Mat3 left_jac1 = left_jac(R1, log_fun1, R2);
        Mat3 left_jac2 = left_jac(R1, log_fun2, R2);
        Mat3 left_jac3 = left_jac(R1, log_fun3, R2);
        Mat3 left_jac4 = left_jac(R1, log_fun4, R2);

        MATRIX_CLOSE(left_jac1, R2.Ad().transpose(), 1e-3);
        MATRIX_CLOSE(left_jac2, -R1.Ad().transpose(), 1e-3);
        MATRIX_CLOSE(left_jac3, -(R2 * R1.inverse()).Ad(), 1e-3);
        MATRIX_CLOSE(left_jac4, Mat3::Identity(), 1e-3);
    }
}

TEST(SO3, VectorJacobians)
{
    const auto passive = [](const SO3<double>& R1, const Vec3& v) { return R1 * v; };
    const auto active = [](const SO3<double>& R1, const Vec3& v) { return R1.inverse() * v; };

    SO3<double> R = SO3<double>::Random();
    Vec3 v = Vec3::Random();

    Mat3 right1 = right_jac2(R, passive, v);
    Mat3 right2 = right_jac2(R, active, v);

    MATRIX_CLOSE(right1, -R * skew(v), 1e-3);
    MATRIX_CLOSE(right2, skew(R.transpose() * v), 1e-3);

    Mat3 left1 = left_jac2(R, passive, v);
    Mat3 left2 = left_jac2(R, active, v);

    MATRIX_CLOSE(left1, -skew(R * v), 1e-3);
    MATRIX_CLOSE(left2, R.transpose() * skew(v), 1e-3);
}

TEST(SO3, ExpLogJacobians)
{
    const auto exp = [](const Vec3& v) { return SO3<double>::exp(v); };
    const auto log = [](const SO3<double>& R) { return R.log(); };
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        Vec3 v = get_new_sample();

        if (i < NUM_ITERS / 3)
        {
            // Close to zero
            v *= 1e-6 / v.norm();
        }
        else if (i < 3. * NUM_ITERS / 6.0)
        {
            // Just above π
            v *= (M_PI + 1e-8) / v.norm();
        }
        else if (i < 4. * NUM_ITERS / 6.0)
        {
            // Just below π
            v *= (M_PI - 1e-8) / v.norm();
        }
        SO3<double> R = SO3<double>::exp(v);

        Mat3 exp_left = left_jac3(v, exp);
        Mat3 jac_exp;
        SO3<double>::exp(v, &jac_exp);

        MATRIX_CLOSE(jac_exp, exp_left, 1e-6);

        Mat3 log_left = left_jac2(R, log);
        Mat3 jac_log;
        R.log(&jac_log);

        MATRIX_CLOSE(jac_log, log_left, 1e-6);

        Mat3 exp_right = right_jac3(v, exp);
        SO3<double>::exp(v, &jac_exp);

        MATRIX_CLOSE(jac_exp.transpose(), exp_right, 1e-6);

        Mat3 log_right = right_jac2(R, log);

        MATRIX_CLOSE(jac_log.transpose(), log_right, 1e-6);
    }
}
