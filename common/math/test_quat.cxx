#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <iostream>
#include <random>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/math/quat.h"
#include "common/matrix_defs.h"
#include "common/numerical_jacobian.h"
#include "common/test_helpers.h"

namespace mc {
namespace math {

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
    MAT_EQ(qR, R_true);
    MAT_EQ(qR.transpose() * v, v_active_rotated);
    MAT_EQ(R_true.transpose() * v, v_active_rotated);

    MAT_EQ(v_x_45, v_active_rotated);

    const Vec3 v_passive_rotated(0, std::pow(0.5, 0.5), std::pow(0.5, 0.5));
    const Vec3 v_x_45_T = q_x_45.rotp(v);
    MAT_EQ(v_x_45_T, v_passive_rotated);
    MAT_EQ(qR * v, v_passive_rotated);
    MAT_EQ(R_true * v, v_passive_rotated);
}

TEST(Quat, quat_rot_invrot_R)
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const Vec3 v = Vec3::Random();
        const Quat<double> q1 = Quat<double>::Random();

        // Check that rotations are inverses of each other
        MAT_EQ(q1.rota(v), q1.R().transpose() * v);
        MAT_EQ(q1.rotp(v), q1.R() * v);
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
        MAT_EQ(q1.rota(v), (q1 * v_pure * q1.inverse()).bar());
        MAT_EQ(q1.rotp(v), (q1.inverse() * v_pure * q1).bar());
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

        MAT_EQ(q.rotp(v1), v2);
        MAT_EQ(q.rota(v2), v1);
    }
}

TEST(Quat, from_R)
{
    Vec3 v;
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const Quat<double> q1 = Quat<double>::Random();
        const SO3<double> R = q1.R();
        const Quat<double> qR = Quat<double>::from_R(R);
        v.setRandom();
        MAT_EQ(qR.rota(v), R.transpose() * v);
        MAT_EQ(q1.rota(v), R.transpose() * v);
        MAT_EQ(qR.rotp(v), R * v);
        MAT_EQ(q1.rotp(v), R * v);
        SO3_EQUALS(R, qR.R());
        QUAT_EQ(qR, q1);
    }
}

TEST(Quat, otimes)
{
    const Quat<double> q1 = Quat<double>::Random();
    const Quat<double> qI = Quat<double>::Identity();
    QUAT_EQ(q1 * q1.inverse(), qI);
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
        QUAT_EQ(q_R_omega_exp, q_omega);
        QUAT_EQ(q_omega_exp, q_omega);

        // Check that exp and log are inverses of each otherprint_error
        MAT_EQ(Quat<double>::exp(omega).log(), omega);
        QUAT_EQ(Quat<double>::exp(q_omega.log()), q_omega);
    }
}

TEST(Quat, inplace_and_mul)
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        const Quat<double> q = Quat<double>::Random();
        Quat<double> q2 = Quat<double>::Random();
        Quat<double> q_copy = q.copy();
        QUAT_EQ(q_copy, q);

        q_copy = q.copy();
        q_copy *= q2;
        QUAT_EQ(q_copy, q * q2);
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
        QUAT_EQ(q, q2);
    }
}

TEST(Quat, euler_singularity)
{
    const double pitch = M_PI / 2.0;
    Quat<double> q = Quat<double>::from_euler(0, pitch, 0);
    q.y() += 1e-8;  // Make the quaternion invalid on purpose to see what happens

    std::cout << q << std::endl;

    EXPECT_NEAR(pitch, q.pitch(), 1e-8);
    const Quat<double> q2 = Quat<double>::from_euler(q.roll(), q.pitch(), q.yaw());
    QUAT_EQ(q, q2);
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

    const Mat3 a = -(q0.R().transpose() * skew(v));  // -R(q).T * [v]
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
        MAT_EQ(delta, delta_recovered);
    }
}

TEST(Quat, make_pure)
{
    const Vec3 v = Vec3::Random();
    const Quat<double> q_pure = Quat<double>::make_pure(v);
    const Vec4 expected(0, v(0), v(1), v(2));
    MAT_EQ(expected, q_pure.elements());
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

    QUAT_EQ(q * Quat<double>::exp(v), Quat<double>::exp(q.Ad() * v) * q);
    MAT_EQ((q * q2).Ad(), q.Ad() * q2.Ad());
}

TEST(Quat, GroupJacobians)
{
    const auto log_fun1 = [](const Quat<double>& x, const Quat<double>& y) { return (y * x); };
    const auto log_fun2 = [](const Quat<double>& x, const Quat<double>& y) {
        return (x.inverse() * y);
    };
    const auto log_fun3 = [](const Quat<double>& x, const Quat<double>& y) {
        return (y * x.inverse());
    };
    const auto log_fun4 = [](const Quat<double>& x, const Quat<double>& y) { return (x * y); };

    for (int i = 0; i < NUM_ITERS; ++i)
    {
        Quat<double> x = Quat<double>::Random();
        Quat<double> y = Quat<double>::Random();

        Mat3 right_jac1 = right_jac(x, log_fun1, y);
        Mat3 right_jac2 = right_jac(x, log_fun2, y);
        Mat3 right_jac3 = right_jac(x, log_fun3, y);
        Mat3 right_jac4 = right_jac(x, log_fun4, y);

        MATRIX_CLOSE(right_jac1, Mat3::Identity(), 1e-3);
        MATRIX_CLOSE(right_jac2, -(x.inverse() * y).Ad().transpose(), 1e-3);
        MATRIX_CLOSE(right_jac3, -x.Ad(), 1e-3);
        MATRIX_CLOSE(right_jac4, y.Ad().transpose(), 1e-3);

        Mat3 left_jac1 = left_jac(x, log_fun1, y);
        Mat3 left_jac2 = left_jac(x, log_fun2, y);
        Mat3 left_jac3 = left_jac(x, log_fun3, y);
        Mat3 left_jac4 = left_jac(x, log_fun4, y);

        MATRIX_CLOSE(left_jac1, y.Ad(), 1e-3);
        MATRIX_CLOSE(left_jac2, -x.Ad().transpose(), 1e-3);
        MATRIX_CLOSE(left_jac3, -(y * x.inverse()).Ad(), 1e-3);
        MATRIX_CLOSE(left_jac4, Mat3::Identity(), 1e-3);
    }
}

TEST(Quat, VectorJacobians)
{
    const auto passive = [](const Quat<double>& x, const Vec3& v) { return x.rotp(v); };
    const auto active = [](const Quat<double>& x, const Vec3& v) { return x.rota(v); };

    Quat<double> x = Quat<double>::Random();
    Vec3 v = Vec3::Random();

    Mat3 right1 = right_jac2(x, passive, v);
    Mat3 right2 = right_jac2(x, active, v);

    MATRIX_CLOSE(right1, skew(x.Ad().inverse() * v), 1e-3);
    MATRIX_CLOSE(right2, -x.Ad() * skew(v), 1e-3);

    Mat3 left1 = left_jac2(x, passive, v);
    Mat3 left2 = left_jac2(x, active, v);

    MATRIX_CLOSE(left1, x.Ad().inverse() * skew(v), 1e-3);
    MATRIX_CLOSE(left2, -skew(x.Ad() * v), 1e-3);
}

TEST(Quat, ExpLogJacobians)
{
    const auto exp = [](const Vec3& v) { return Quat<double>::exp(v); };
    const auto log = [](const Quat<double>& R) { return R.log(); };

    Quat<double> R = Quat<double>::Random();
    Vec3 v = Vec3::Random();

    Mat3 left_jac_exp;
    Quat<double>::exp<JacobianSide::LEFT>(v, &left_jac_exp);
    Mat3 right_jac_exp;
    Quat<double>::exp<JacobianSide::RIGHT>(v, &right_jac_exp);

    MATRIX_CLOSE(left_jac_exp, left_jac3(v, exp), 1e-6);
    MATRIX_CLOSE(right_jac_exp, right_jac3(v, exp), 1e-6);

    Mat3 left_jac_log;
    R.log<JacobianSide::LEFT>(&left_jac_log);
    Mat3 right_jac_log;
    R.log<JacobianSide::RIGHT>(&right_jac_log);

    MATRIX_CLOSE(left_jac_log, left_jac2(R, log), 1e-6);
    MATRIX_CLOSE(right_jac_log, right_jac2(R, log), 1e-6);

    MAT_EQ(right_jac_log, left_jac_log.transpose());
    MAT_EQ(right_jac_exp, left_jac_exp.transpose());
}

TEST(Quat, SO3Equivalent)
{
    // Exp same
    const Vec3 w = Vec3::Random();
    Quat<double> q = Quat<double>::exp(w);
    SO3<double> R = SO3<double>::exp(w);
    SO3_EQUALS(q.R(), R.transpose());
    QUAT_EQ(q, R.q().inverse());

    // Log same
    MAT_EQ(R.log(), q.log());
    MAT_EQ(R.log(), w);

    // rotate vector same
    q = Quat<double>::Random();
    R = q.R();
    const Vec3 v = Vec3::Random();
    MAT_EQ(R * v, q.rotp(v));
    MAT_EQ(R.transpose() * v, q.rota(v));

    // concanate correctly
    const Quat<double> qa2b = Quat<double>::Random();
    const Quat<double> qb2c = Quat<double>::Random();
    const SO3<double> Ra2b = qa2b.R();
    const SO3<double> Rb2c = qb2c.R();

    const SO3<double> Ra2c = Rb2c * Ra2b;
    const Quat<double> qa2c = qa2b * qb2c;

    SO3_EQUALS(Ra2c, qa2c.R());
    QUAT_EQ(Ra2c.q(), qa2c);
}

TEST(Quat, dGen_dParam)
{
    const Quat<double> Q1 = Quat<double>::Random();
    const auto fun = [Q1](const Vec4& q) -> Vec3 {
        return (Q1.inverse() * Quat<double>(q.data())).log();
    };

    const Quat<double> Q2 = Q1;

    const Eigen::Matrix<double, 3, 4> jac_numerical = compute_jac(Vec4(Q2.arr_), fun);
    const Eigen::Matrix<double, 3, 4> jac_analytical = Q1.dGenDParam();

    MATRIX_CLOSE(jac_numerical, jac_analytical, 1e-6);
}

TEST(Quat, LocalParamExp)
{
    const Quat<double> Q = Quat<double>::Random();
    const auto fun = [Q](const Vec3& delta) -> Vec4 { return (Q * Quat<double>::exp(delta)).arr_; };

    const Vec3 delta = Vec3::Zero();

    const Eigen::Matrix<double, 4, 3> jac_numerical = compute_jac(delta, fun);
    const Eigen::Matrix<double, 4, 3> jac_analytical = Q.dParamDGen();

    MATRIX_CLOSE(jac_numerical, jac_analytical, 1e-6);
}

TEST(Quat, dGenDParam_dParamDGen)
{
    const Quat<double> q = Quat<double>::Identity();

    const Mat4 jac = q.dParamDGen() * q.dGenDParam();
    Mat4 answer = Mat4::Zero();
    answer.bottomRightCorner<3, 3>().setIdentity();
    MATRIX_CLOSE(jac, answer, 1e-8);

    const Mat3 jac2 = q.dGenDParam() * q.dParamDGen();
    MATRIX_CLOSE(jac2, Mat3::Identity(), 1e-8);
}

}  // namespace math
}  // namespace mc
