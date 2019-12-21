#include <iostream>
#include <random>

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/geometry/quat.h"
#include "common/matrix_defs.h"
#include "common/test_helpers.h"

constexpr int NUM_ITERS = 100;

TEST(Quat, quat_rotation_direction)
{
    // Compare against a known active and passive rotation
    const Vec3 v(0, 0, 1);
    const Vec3 beta(1, 0, 0);
    Quat<double> q_x_45 = Quat<double>::from_axis_angle(beta, 45.0 * M_PI / 180.0);
    const Vec3 v_x_45 = q_x_45.rota(v);
    const Vec3 v_active_rotated(0, -1.0 * std::pow(0.5, 0.5), std::pow(0.5, 0.5));

    // clang-format off
    Mat3 R_true;
    R_true << 1.0000000,  0.0000000,           0.0000000,
              0.0000000,  0.70710678118654757, 0.70710678118654757,
              0.0000000, -0.70710678118654757, 0.70710678118654757;
    // clang-format on
    const Mat3 qR = q_x_45.R();
    MATRIX_EQUALS(qR, R_true);
    MATRIX_EQUALS(qR.transpose() * v, v_active_rotated);
    MATRIX_EQUALS(R_true.transpose() * v, v_active_rotated);

    MATRIX_EQUALS(v_x_45, v_active_rotated);

    const Vec3 v_passive_rotated(0, std::pow(0.5, 0.5), std::pow(0.5, 0.5));
    const Vec3 v_x_45_T = q_x_45.rotp(v);
    MATRIX_EQUALS(v_x_45_T, v_passive_rotated);
    MATRIX_EQUALS(qR * v, v_passive_rotated);
    MATRIX_EQUALS(R_true * v, v_passive_rotated);
}

TEST(Quat, quat_rot_invrot_R)
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const Vec3 v = Vec3::Random();
        const Quat<double> q1 = Quat<double>::Random();

        // Check that rotations are inverses of each other
        MATRIX_EQUALS(q1.rota(v), q1.R().transpose() * v);
        MATRIX_EQUALS(q1.rotp(v), q1.R() * v);
    }
}

TEST(Quat, quat_rot_invrot_otimes)
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const Vec3 v = Vec3::Random();
        const Quat<double> q1 = Quat<double>::Random();
        const Quat<double> v_pure = Quat<double>::make_pure(v);

        // Check that I can use otimes to rotate a vector
        MATRIX_EQUALS(q1.rota(v), (q1 * v_pure * q1.inverse()).bar());
        MATRIX_EQUALS(q1.rotp(v), (q1.inverse() * v_pure * q1).bar());
    }
}

TEST(Quat, quat_from_two_unit_vectors)
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

        const Quat<double> q = Quat<double>::from_two_unit_vectors(v1, v2);

        MATRIX_EQUALS(q.rotp(v1), v2);
        MATRIX_EQUALS(q.rota(v2), v1);
    }
}

TEST(Quat, from_R)
{
    Vec3 v;
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const Quat<double> q1 = Quat<double>::Random();
        const Mat3 R = q1.R();
        const Quat<double> qR = Quat<double>::from_R(R);
        v.setRandom();
        MATRIX_EQUALS(qR.rota(v), R.transpose() * v);
        MATRIX_EQUALS(q1.rota(v), R.transpose() * v);
        MATRIX_EQUALS(qR.rotp(v), R * v);
        MATRIX_EQUALS(q1.rotp(v), R * v);
        MATRIX_EQUALS(R, qR.R());
        QUATERNION_EQUALS(qR, q1);
    }
}

TEST(Quat, otimes)
{
    const Quat<double> q1 = Quat<double>::Random();
    const Quat<double> qI = Quat<double>::Identity();
    QUATERNION_EQUALS(q1 * q1.inverse(), qI);
}

TEST(Quat, exp_log_axis_angle)
{
    // Check that qexp is right by comparing with matrix exp and axis-angle
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const Vec3 omega = Vec3::Random();
        const Mat3 R_omega_exp_T = skew(omega).exp();
        const Quat<double> q_R_omega_exp = Quat<double>::from_R(R_omega_exp_T.transpose());
        const Quat<double> q_omega =
            Quat<double>::from_axis_angle(omega / omega.norm(), omega.norm());
        const Quat<double> q_omega_exp = Quat<double>::exp(omega);
        QUATERNION_EQUALS(q_R_omega_exp, q_omega);
        QUATERNION_EQUALS(q_omega_exp, q_omega);

        // Check that exp and log are inverses of each otherprint_error
        MATRIX_EQUALS(Quat<double>::exp(omega).log(), omega);
        QUATERNION_EQUALS(Quat<double>::exp(q_omega.log()), q_omega);
    }
}

TEST(Quat, inplace_and_mul)
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const Quat<double> q = Quat<double>::Random();
        Quat<double> q2 = Quat<double>::Random();
        Quat<double> q_copy = q.copy();
        QUATERNION_EQUALS(q_copy, q);

        q_copy = q.copy();
        q_copy *= q2;
        QUATERNION_EQUALS(q_copy, q * q2);
    }
}

TEST(Quat, euler)
{
    std::uniform_real_distribution<double> uni(-1, 1);
    std::default_random_engine gen;
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const double roll = M_PI * uni(gen);
        const double pitch = M_PI / 2.0 * uni(gen);
        const double yaw = M_PI * uni(gen);
        const Quat<double> q = Quat<double>::from_euler(roll, pitch, yaw);

        EXPECT_NEAR(roll, q.roll(), 1e-8);
        EXPECT_NEAR(pitch, q.pitch(), 1e-8);
        EXPECT_NEAR(yaw, q.yaw(), 1e-8);
        const Quat<double> q2 = Quat<double>::from_euler(q.roll(), q.pitch(), q.yaw());
        QUATERNION_EQUALS(q, q2);
    }
}

TEST(Quat, passive_rotation_derivative)
{
    const Quat<double> q0 = Quat<double>::Random();
    const Vec3 v = Vec3::Random();

    const Mat3 I = Mat3::Identity();
    constexpr double epsilon = 1e-8;

    const Mat3 a = skew(q0.rotp(v));  // [R(q)v]
    Mat3 fd;

    for (int i = 0; i < 3; i++)
    {
        const Quat<double> qi = q0 * Quat<double>::exp(epsilon * (I.col(i)));
        fd.col(i) = (qi.rotp(v) - q0.rotp(v)) / epsilon;
    }
    MATRIX_CLOSE(fd, a, 1e-6);
}

TEST(Quat, active_rotation_derivative)
{
    const Quat<double> q0 = Quat<double>::Random();
    const Vec3 v = Vec3::Random();

    const Mat3 I = Mat3::Identity();
    const double epsilon = 1e-8;

    const Mat3 a = -q0.R().transpose() * skew(v);  // -R(q).T * [v]
    Mat3 fd;

    for (int i = 0; i < 3; i++)
    {
        const Quat<double> qi = q0 * Quat<double>::exp(epsilon * (I.col(i)));
        fd.col(i) = (qi.rota(v) - q0.rota(v)) / epsilon;
    }
    MATRIX_CLOSE(fd, a, 1e-6);
}

TEST(Quat, exp_approx)
{
    const Mat3 I = Mat3::Identity();
    for (int j = 0; j < NUM_ITERS; j++)
    {
        Quat<double> q = Quat<double>::Random();
        if (j == 0)
            q = Quat<double>::Identity();

        Vec3 delta;
        delta.setRandom();
        delta *= 0.1;

        const Quat<double> qp = q * Quat<double>::exp(delta);

        const Mat3 actual = qp.R();
        const Mat3 approx = (I - skew(delta)) * q.R();
        MATRIX_CLOSE(actual, approx, 0.1);
    }
}

TEST(Quat, log_close)
{
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        const Quat<double> q = Quat<double>::Random();
        const Vec3 delta = Vec3::Random() * 4e-6;
        const Quat<double> q2 = q * Quat<double>::exp(delta);
        const Vec3 delta_recovered = (q.inverse() * q2).log();
        MATRIX_EQUALS(delta, delta_recovered);
    }
}

TEST(Quat, make_pure)
{
    const Vec3 v = Vec3::Random();
    const Quat<double> q_pure = Quat<double>::make_pure(v);
    const Vec4 expected(0, v(0), v(1), v(2));
    MATRIX_EQUALS(expected, q_pure.elements());
}

TEST(Quat, check_dynamics)
{
    Vec3 v(1, 0, 0);
    const Quat<double> q_I2a = Quat<double>::from_euler(0, 0.5, 0);
    const Quat<double> q_I2b = q_I2a * Quat<double>::exp(v);
    EXPECT_FLOAT_EQ(q_I2b.roll(), 1.0);
    EXPECT_FLOAT_EQ(q_I2b.pitch(), 0.5);

    const Quat<double> q_a2I = Quat<double>::from_euler(0, -0.5, 0);
    const Quat<double> q_b2I = Quat<double>::exp(-v) * q_a2I;

    EXPECT_FLOAT_EQ(q_b2I.inverse().roll(), 1.0);
    EXPECT_FLOAT_EQ(q_b2I.inverse().pitch(), 0.5);
}

TEST(Quat, Adjoint)
{
    for (int j = 0; j < NUM_ITERS; ++j)
    {
        constexpr double eps = 1e-8;
        const Quat<double> q = Quat<double>::Random();
        Mat3 fd = Mat3::Zero();
        for (int i = 0; i < 3; ++i)
        {
            fd.col(i) = (q * Quat<double>::exp(Vec3::Unit(i) * eps) * q.inverse()).log() / eps;
        }

        MATRIX_CLOSE(fd, q.Ad(), 1e-3);
    }
}

TEST(Quat, AdjointIdentities)
{
    const Quat<double> q = Quat<double>::Random();
    const Quat<double> q2 = Quat<double>::Random();
    const Vec3 v = Vec3::Random();

    QUATERNION_EQUALS(q * Quat<double>::exp(v), Quat<double>::exp(q.Ad() * v) * q);
    MATRIX_EQUALS((q * q2).Ad(), q.Ad() * q2.Ad());
}
