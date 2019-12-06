#include <iostream>
#include <random>

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/geometry/quat.h"
#include "common/test_helpers.h"

using namespace Eigen;
using namespace std;

constexpr int NUM_ITERS = 100;

TEST(Quat, quat_rotation_direction)
{
    // Compare against a known active and passive rotation
    Vector3d v, beta, v_active_rotated, v_passive_rotated;
    v << 0, 0, 1;
    v_active_rotated << 0, -1.0 * std::pow(0.5, 0.5), std::pow(0.5, 0.5);
    beta << 1, 0, 0;
    Quatd q_x_45 = Quatd::from_axis_angle(beta, 45.0 * M_PI / 180.0);
    Eigen::Vector3d v_x_45 = q_x_45.rota(v);

    Matrix3d R_true;
    R_true << 1.0000000, 0.0000000, 0.0000000, 0.0000000, 0.70710678118654757, 0.70710678118654757,
        0.0000000, -0.70710678118654757, 0.70710678118654757;
    Matrix3d qR = q_x_45.R();
    MATRIX_EQUALS(qR, R_true);
    MATRIX_EQUALS(qR.transpose() * v, v_active_rotated);
    MATRIX_EQUALS(R_true.transpose() * v, v_active_rotated);

    MATRIX_EQUALS(v_x_45, v_active_rotated);

    v_passive_rotated << 0, std::pow(0.5, 0.5), std::pow(0.5, 0.5);
    Vector3d v_x_45_T = q_x_45.rotp(v);
    MATRIX_EQUALS(v_x_45_T, v_passive_rotated);
    MATRIX_EQUALS(qR * v, v_passive_rotated);
    MATRIX_EQUALS(R_true * v, v_passive_rotated);
}

TEST(Quat, quat_rot_invrot_R)
{
    Vector3d v;
    Quatd q1 = Quatd::Random();
    for (int i = 0; i < NUM_ITERS; i++)
    {
        v.setRandom();
        q1 = Quatd::Random();

        // Check that rotations are inverses of each other
        MATRIX_EQUALS(q1.rota(v), q1.R().transpose() * v);
        MATRIX_EQUALS(q1.rotp(v), q1.R() * v);
    }
}

TEST(Quat, quat_from_two_unit_vectors)
{
    Vector3d v1, v2;
    for (int i = 0; i < NUM_ITERS; i++)
    {
        v1.setRandom();
        v2.setRandom();
        v1 /= v1.norm();
        v2 /= v2.norm();

        MATRIX_EQUALS(Quatd::from_two_unit_vectors(v1, v2).rota(v1), v2);
        MATRIX_EQUALS(Quatd::from_two_unit_vectors(v2, v1).rotp(v1), v2);
    }
}

TEST(Quat, from_R)
{
    Vector3d v;
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Quatd q1 = Quatd::Random();
        Matrix3d R = q1.R();
        Quatd qR = Quatd::from_R(R);
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
    Quatd q1 = Quatd::Random();
    Quatd qI = Quatd::Identity();
    QUATERNION_EQUALS(q1 * q1.inverse(), qI);
}

TEST(Quat, exp_log_axis_angle)
{
    // Check that qexp is right by comparing with matrix exp and axis-angle
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Vector3d omega;
        omega.setRandom();
        Matrix3d R_omega_exp_T = Quatd::skew(omega).exp();  // Why is this needed?
        Quatd q_R_omega_exp = Quatd::from_R(R_omega_exp_T.transpose());
        Quatd q_omega = Quatd::from_axis_angle(omega / omega.norm(), omega.norm());
        Quatd q_omega_exp = Quatd::exp(omega);
        QUATERNION_EQUALS(q_R_omega_exp, q_omega);
        QUATERNION_EQUALS(q_omega_exp, q_omega);

        // Check that exp and log are inverses of each otherprint_error
        MATRIX_EQUALS(Quatd::log(Quatd::exp(omega)), omega);
        QUATERNION_EQUALS(Quatd::exp(Quatd::log(q_omega)), q_omega);
    }
}

TEST(Quat, inplace_and_mul)
{
    Vector3d delta1, delta2, zeros;
    zeros.setZero();
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Quatd q = Quatd::Random();
        Quatd q2 = Quatd::Random();
        Quatd q_copy = q.copy();
        QUATERNION_EQUALS(q_copy, q);
        delta1.setRandom();
        delta2.setRandom();

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
        double roll = M_PI * uni(gen);
        double pitch = M_PI / 2.0 * uni(gen);
        double yaw = M_PI * uni(gen);
        Quatd q = Quatd::from_euler(roll, pitch, yaw);
        ASSERT_NEAR(roll, q.roll(), 1e-8);
        ASSERT_NEAR(pitch, q.pitch(), 1e-8);
        ASSERT_NEAR(yaw, q.yaw(), 1e-8);
        Quatd q2 = Quatd::from_euler(q.roll(), q.pitch(), q.yaw());
        QUATERNION_EQUALS(q, q2);
    }
}

TEST(Quat, passive_rotation_derivative)
{
    Quatd q0 = Quatd::Random();
    Vector3d v;
    v.setRandom();

    Matrix3d a;
    Matrix3d fd;
    Matrix3d I = Matrix3d::Identity();
    double epsilon = 1e-8;

    a = skew(q0.rotp(v));  // [R(q)v]

    for (int i = 0; i < 3; i++)
    {
        Quatd qi = q0 * Quatd::exp(epsilon * (I.col(i)));
        fd.col(i) = (qi.rotp(v) - q0.rotp(v)) / epsilon;
    }
    if ((fd - a).array().abs().sum() > 1e-6)
    {
        std::cout << "ERROR IN LINE " << __LINE__ << "\nA:\n" << a << "\nD:\nfd" << fd << std::endl;
    }
    ASSERT_LE((fd - a).array().abs().sum(), 1e-6);
}

TEST(Quat, active_rotation_derivative)
{
    Quatd q0 = Quatd::Random();
    Vector3d v;
    v.setRandom();

    Matrix3d a;
    Matrix3d fd;
    Matrix3d I = Matrix3d::Identity();
    double epsilon = 1e-8;

    a = -q0.R().transpose() * skew(v);  // -R(q).T * [v]

    for (int i = 0; i < 3; i++)
    {
        Quatd qi = q0 * Quatd::exp(epsilon * (I.col(i)));
        fd.col(i) = (qi.rota(v) - q0.rota(v)) / epsilon;
    }
    if ((fd - a).array().abs().sum() > 1e-6)
    {
        std::cout << "ERROR IN LINE " << __LINE__ << "\nA:\n" << a << "\nD:\nfd" << fd << std::endl;
    }
    ASSERT_LE((fd - a).array().abs().sum(), 1e-6);
}

TEST(Quat, exp_approx)
{
    for (int j = 0; j < NUM_ITERS; j++)
    {
        Quatd q = Quatd::Random();
        if (j == 0)
            q = Quatd::Identity();
        Vector3d delta;
        Matrix3d I = Matrix3d::Identity();
        delta.setRandom();
        delta *= 0.1;

        Quatd qp = q * Quatd::exp(delta);

        Matrix3d actual = qp.R();
        Matrix3d approx = (I - skew(delta)) * q.R();
        ASSERT_LE((actual - approx).array().abs().sum(), 0.1);
    }
}
