#include <iostream>
#include <random>

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/geometry/dquat.h"
#include "common/geometry/se3.h"
#include "common/matrix_defs.h"
#include "common/numerical_jacobian.h"
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

TEST(DQuat, Adjoint)
{
    for (int j = 0; j < NUM_ITERS; ++j)
    {
        constexpr double eps = 1e-8;
        const DQuat<double> q = DQuat<double>::Random();
        Mat6 fd;
        for (int i = 0; i < 6; ++i)
        {
            fd.col(i) = (q * DQuat<double>::exp(Vec6::Unit(i) * eps) * q.inverse()).log() / eps;
        }

        MATRIX_CLOSE(fd, q.Ad(), 1e-3);
    }
}

TEST(DQuat, AdjointIdentities)
{
    const DQuat<double> q = DQuat<double>::Random();
    const DQuat<double> q2 = DQuat<double>::Random();
    const Vec6 v = Vec6::Random();

    DQUAT_EQUALS(q * DQuat<double>::exp(v), DQuat<double>::exp(q.Ad() * v) * q);
    MATRIX_EQUALS((q * q2).Ad(), q.Ad() * q2.Ad());
}

TEST(DQuat, GroupJacobians)
{
    const auto log_fun1 = [](const DQuat<double>& x, const DQuat<double>& y) { return (y * x); };
    const auto log_fun2 = [](const DQuat<double>& x, const DQuat<double>& y) {
        return (x.inverse() * y);
    };
    const auto log_fun3 = [](const DQuat<double>& x, const DQuat<double>& y) {
        return (y * x.inverse());
    };
    const auto log_fun4 = [](const DQuat<double>& x, const DQuat<double>& y) { return (x * y); };

    for (int i = 0; i < NUM_ITERS; ++i)
    {
        DQuat<double> x = DQuat<double>::Random();
        DQuat<double> y = DQuat<double>::Random();

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

TEST(DQuat, VectorJacobians)
{
    const auto passive = [](const DQuat<double>& T, const Vec3& v) { return T.transformp(v); };
    const auto active = [](const DQuat<double>& T, const Vec3& v) { return T.transforma(v); };

    typedef Eigen::Matrix<double, 3, 6> Mat36;
    DQuat<double> q = DQuat<double>::Random();
    // q.real() = Quat<double>::Identity();
    // q.dual().arr_.setZero();
    Vec3 v = Vec3::Random();

    Mat36 right1 = right_jac2(q, passive, v);
    Mat36 right2 = right_jac2(q, active, v);

    Mat36 a;
    a << skew(v), Mat3::Identity();

    MATRIX_CLOSE(right1, hstack(skew(q.R() * (v - q.translation())), -Mat3::Identity()), 1e-3);
    MATRIX_CLOSE(right2, hstack(q.R().inverse() * skew(-v), q.R().inverse()), 1e-3);

    Mat36 left1 = left_jac2(q, passive, v);
    Mat36 left2 = left_jac2(q, active, v);
    MATRIX_CLOSE(left1, hstack(q.R() * skew(v), -q.R()), 1e-3);
    MATRIX_CLOSE(left2, hstack(-skew(q.R().transpose() * v + q.translation()), Mat3::Identity()),
                 1e-3);
}

TEST(DQuat, ExpLogJacobians)
{
    const auto exp = [](const Vec6& v) { return DQuat<double>::exp(v); };
    const auto log = [](const DQuat<double>& T) { return T.log(); };

    for (int i = 0; i < NUM_ITERS; ++i)
    {
        DQuat<double> T = DQuat<double>::Random();
        const Vec6 v = Vec6::Random();

        const Mat6 exp_left = left_jac3(v, exp);
        Mat6 jac_exp;
        T = DQuat<double>::exp(v, &jac_exp);

        MATRIX_CLOSE(jac_exp, exp_left, 1e-6);

        Mat6 jac2_exp;
        SE3<double> T_SE3 = SE3<double>::exp(v, &jac2_exp);

        const Mat6 log_left = left_jac2(T, log);
        Mat6 jac_log;
        T.log(&jac_log);

        Mat6 jac2_log;
        SE3<double> S(T.R(), -T.R().transpose() * T.translation());
        S.log(&jac2_log);
        SO3_EQUALS(T.R(), S.R());

        MATRIX_CLOSE(jac_log, log_left, 1e-6);

        Mat6 other_log;
        T_SE3.inverse().log(&other_log);

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
}
