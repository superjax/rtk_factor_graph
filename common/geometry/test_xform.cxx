#include <random>

#include <gtest/gtest.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "common/geometry/xform.h"
#include "common/matrix_defs.h"
#include "common/test_helpers.h"

constexpr int NUM_ITERS = 100;

TEST(xform, known_transform)
{
    Xformd T_known((Vec3() << 1, 1, 0).finished(),
                   Quatd::from_axis_angle((Vec3() << 0, 0, 1).finished(), M_PI / 4.0));
    Xformd T_known_inv((Vec3() << -sqrt(2), 0, 0).finished(),
                       Quatd::from_axis_angle((Vec3() << 0, 0, 1).finished(), -M_PI / 4.0));
    TRANSFORM_EQUALS(T_known.inverse(), T_known_inv);
}

TEST(xform, ToMatrix)
{
    Xformd X1 = Xformd::Random();
    Mat4 mat = X1.Mat();
    Xformd X2(mat);
    TRANSFORM_EQUALS(X1, X2);
}

TEST(xform, known_vector_passive_transform)
{
    Vec3 p1;
    p1 << 1, 0, 0;
    Xformd T_known((Vec3() << 1, 1, 0).finished(),
                   Quatd::from_axis_angle((Vec3() << 0, 0, 1).finished(), M_PI / 4.0));
    Vec3 p2;
    p2 << -sqrt(0.5), -sqrt(0.5), 0;
    MATRIX_EQUALS(p2, T_known.transformp(p1));
}

TEST(xform, known_vector_active_transform)
{
    Vec3 p1;
    p1 << 1, 0, 0;
    Xformd T_known((Vec3() << 1, 1, 0).finished(),
                   Quatd::from_axis_angle((Vec3() << 0, 0, 1).finished(), M_PI / 4.0));
    Vec3 p2;
    p2 << 1 + sqrt(0.5), 1 + sqrt(0.5), 0;
    MATRIX_EQUALS(p2, T_known.transforma(p1));
}

TEST(xform, inverse)
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Xformd T1 = Xformd::Random();
        Xformd T2 = T1.inverse();
        Xformd T3 = T1 * T2;
        QUATERNION_EQUALS(T3.q(), Quatd::Identity());
        EXPECT_NEAR(T3.t().norm(), 0, 1e-8);
    }
}

TEST(xform, transform_vector)
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Xformd T1 = Xformd::Random();
        Vec3 p;
        p.setRandom();
        MATRIX_EQUALS(T1.transformp(T1.inverse().transformp(p)), p);
        MATRIX_EQUALS(T1.inverse().transformp(T1.transformp(p)), p);
        MATRIX_EQUALS(T1.transforma(T1.inverse().transforma(p)), p);
        MATRIX_EQUALS(T1.inverse().transforma(T1.transforma(p)), p);
    }
}

TEST(xform, exp)
{
    Vec3 v;
    Vec3 omega;
    for (int i = 0; i < NUM_ITERS; i++)
    {
        v.setRandom();
        omega.setRandom();

        Xformd x = Xformd::Identity();
        double dt = 0.0001;
        for (double t = 0; t < 1.0; t += dt)
        {
            x.t_ += x.q_.rota(v * dt);
            x.q_ += omega * dt;
        }

        Vec6 delta;
        delta << v, omega;
        Xformd xexp = Xformd::exp(delta);
        TRANSFORM_CLOSE(x, xexp, 1.5e-4);
    }

    for (int i = 0; i < NUM_ITERS; i++)
    {
        v.setRandom();
        omega.setRandom();

        Xformd x = Xformd::Random();
        Xformd x0 = x;
        double dt = 0.0001;
        for (double t = 0; t < 1.0; t += dt)
        {
            x.t_ += x.q_.rota(v * dt);
            x.q_ += omega * dt;
        }

        Vec6 delta;
        delta << v, omega;
        Xformd xexp = x0 * Xformd::exp(delta);
        TRANSFORM_CLOSE(x, xexp, 1.5e-4);
    }
}

TEST(xform, log_exp)
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Vec6 xi;
        xi.setRandom();
        MATRIX_EQUALS(Xformd::log(Xformd::exp(xi)), xi);
    }

    for (int i = 0; i < NUM_ITERS; i++)
    {
        Xformd x = Xformd::Random();
        TRANSFORM_CLOSE(Xformd::exp(Xformd::log(x)), x, 1e-8);
    }
}

TEST(xform, boxplus_boxminus)
{
    for (int i = 0; i < NUM_ITERS; i++)
    {
        Xformd T = Xformd::Random();
        Xformd T2 = Xformd::Random();
        Vec6 zeros, dT, dT2;
        zeros.setZero();
        dT.setRandom();
        dT2.setRandom();
        // TRANSFORM_EQUALS(T + zeros, T);
        // TRANSFORM_EQUALS(T + (T2 - T), T2);
        // MATRIX_EQUALS((T + dT) - T, dT);
        TRANSFORM_EQUALS(T * Xformd::exp(zeros), T);
        TRANSFORM_EQUALS(T * Xformd::exp(Xformd::log(T.inverse() * T2)), T2);
        MATRIX_EQUALS(Xformd::log(T.inverse() * (T * Xformd::exp(dT))), dT);
        // EXPECT_LE(Xformd::log((Xformd::exp(dT2)).inverse() * (Xformd::exp(dT))).norm(),
        //   (dT - dT2).norm());
        // EXPECT_LE(((T + dT) - (T + dT2)).norm(), (dT - dT2).norm());
    }
}
