#include <iostream>
#include <random>

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/geometry/dquat.h"
#include "common/matrix_defs.h"
#include "common/test_helpers.h"

constexpr int NUM_ITERS = 100;

TEST(DQuat, compile)
{
    const DQuat<double> Q;
    for (int i = 0; i < 8; ++i)
    {
        EXPECT_FLOAT_EQ(Q[i], (i == 0) ? 1 : 0);
    }
}

TEST(DQuat, mapped_memory)
{
    double buffer[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    DQuat<double> Q(buffer);
    for (int i = 0; i < 8; ++i)
    {
        EXPECT_FLOAT_EQ(Q[i], i + 1);
        Q[i] = -42;
        EXPECT_FLOAT_EQ(buffer[i], -42);
    }
}

TEST(DQuat, FromQuatAndTranslation)
{
    const Quat<double> q = Quat<double>::from_axis_angle(Vec3(0, 0, 1), M_PI / 4.0);
    const Vec3 t(2, 1, 0);
    DQuat<double> Q(q, t);
    QUATERNION_EQUALS(q, Q.r_);
    QUATERNION_EQUALS(0.5 * (Quat<double>::make_pure(t) * q), Q.d_);
}

TEST(DQuat, Concatenate)
{
    const Quat<double> q_a2b = Quat<double>::from_axis_angle(Vec3(0, 0, 1), M_PI / 4.0);
    const Vec3 t_a2b(2, 1, 0);
    const Quat<double> q_b2c = Quat<double>::from_axis_angle(Vec3(0, 0, 1), -M_PI / 3.0);
    const Vec3 t_b2c(2, 0, 0);

    const DQuat<double> Q_a2b(q_a2b, t_a2b);
    const DQuat<double> Q_b2c(q_b2c, t_b2c);

    const DQuat<double> Q_a2c = Q_a2b * Q_b2c;

    QUATERNION_EQUALS(Q_a2c.rotation(), Quat<double>::from_axis_angle(Vec3(0, 0, 1), -M_PI / 12.0));
    MATRIX_CLOSE(Q_a2c.translation(), Vec3(3.414, 2.414, 0), 1e-3);
}

TEST(DQuat, KnownPassiveTransform)
{
    const Quat<double> q_a2b = Quat<double>::from_axis_angle(Vec3(0, 0, 1), M_PI / 4.0);
    const Vec3 t_a(1, 1, 0);
    const Vec3 p_a(1, 0, 0);
    const DQuat<double> Q_a2b(q_a2b, t_a);
    const Vec3 p_b(-sqrt(0.5), -sqrt(0.5), 0);
    MATRIX_EQUALS(p_b, Q_a2b.transformp(p_a));
}

TEST(DQuat, KnownActiveTransform)
{
    const Quat<double> q_a2b = Quat<double>::from_axis_angle(Vec3(0, 0, 1), M_PI / 4.0);
    const Vec3 t_a(1, 1, 0);
    const Vec3 p_a(1, 0, 0);
    const DQuat<double> Q_a2b(q_a2b, t_a);
    const Vec3 p_b(1 + sqrt(0.5), 1 + sqrt(0.5), 0);
    MATRIX_EQUALS(p_b, Q_a2b.transforma(p_a));
}

TEST(DQuat, inverse)
{
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        const DQuat<double> T1 = DQuat<double>::Random();
        const DQuat<double> T2 = T1.inverse();
        const DQuat<double> T3 = T1 * T2;
        DQUAT_EQUALS(T3, DQuat<double>::identity());
    }
}

TEST(DQuat, transform_vector)
{
    for (int i = 0; i < NUM_ITERS; ++i)
    {
        const DQuat<double> Q1 = DQuat<double>::Random();
        const Vec3 p = Vec3::Random();
        MATRIX_EQUALS(Q1.transformp(Q1.inverse().transformp(p)), p);
        MATRIX_EQUALS(Q1.inverse().transformp(Q1.transformp(p)), p);
        MATRIX_EQUALS(Q1.transforma(Q1.inverse().transforma(p)), p);
        MATRIX_EQUALS(Q1.inverse().transforma(Q1.transforma(p)), p);
    }
}

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

TEST(DQuat, dynamics)
{
    // Check that we can euler integrate Qdot = 1/2 Q * zeta
    const Vec6 omega = Vec6::Random();
    const auto w = omega.head<3>();
    const auto v = omega.tail<3>();
    constexpr int NUM_STEPS = 1000;

    constexpr double eps = 1.0 / (double)NUM_STEPS;
    DQuat<double> Q_numerical = DQuat<double>::identity();
    Vec3 test = Vec3::Zero();
    for (int i = 0; i < NUM_STEPS; ++i)
    {
        test += Q_numerical.real().rota(eps * v);
        const DQuat<double> step(Quat<double>::make_pure(w), Quat<double>::make_pure(v));
        Q_numerical.arr_ += 0.5 * eps * (Q_numerical * step).arr_;
        Q_numerical.real().normalize();
    }
    MATRIX_CLOSE(Q_numerical.translation(), test, 1e-3);
}

TEST(DQuat, exp)
{
    // Check that qexp is right by comparing with an euler integration with tiny steps
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Vec6 omega = Vec6::Random();
        const auto w = omega.head<3>();
        const auto v = omega.tail<3>();

        if (i < NUM_ITERS / 3)
        {
            // Excercise small-angle
            omega *= 1e-5;
        }

        constexpr int NUM_STEPS = 1000;
        constexpr double eps = 1.0 / (double)NUM_STEPS;
        DQuat<double> Q_numerical = DQuat<double>::identity();
        for (int j = 0; j < NUM_STEPS; ++j)
        {
            DQuat<double> step(Quat<double>::make_pure(w), Quat<double>::make_pure(v));
            Q_numerical.arr_ += 0.5 * eps * (Q_numerical * step).arr_;
            Q_numerical.real().normalize();
        }
        const DQuat<double> Q_ours = DQuat<double>::exp(omega);
        MATRIX_CLOSE(Q_numerical.arr_, Q_ours.arr_, 1e-3);
    }
}

TEST(DQuat, log)
{
    // Check that log is the inverse of exp
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Vec6 omega = Vec6::Random();

        if (i < NUM_ITERS / 3)
        {
            // Excercise small-angle
            omega *= 1e-7;
        }

        DQuat<double> Q = DQuat<double>::exp(omega);
        const Vec6 w_ours = Q.log();
        MATRIX_CLOSE(w_ours, omega, 1e-8);
    }
}
