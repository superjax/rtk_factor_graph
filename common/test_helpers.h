#pragma once

#include <cmath>

#define MATRIX_CLOSE(m1, m2, tol)                                 \
    {                                                             \
        for (int row = 0; row < (m1).rows(); row++)               \
        {                                                         \
            for (int col = 0; col < (m1).cols(); col++)           \
            {                                                     \
                EXPECT_NEAR((m1)(row, col), (m2)(row, col), tol); \
            }                                                     \
        }                                                         \
    }

#define QUATERNION_EQUALS(q1, q2) \
    MATRIX_CLOSE((q1).arr_, (sign((q2).w()) * sign((q1).w())) * (q2).arr_, 1e-8)

#define MATRIX_EQUALS(v1, v2) MATRIX_CLOSE(v1, v2, 1e-8)

#define TRANSFORM_EQUALS(t1, t2)       \
    MATRIX_EQUALS((t1).t(), (t2).t()); \
    QUATERNION_EQUALS((t1).q(), (t2).q())

#define TRANSFORM_CLOSE(t1, t2, tol)                                                 \
    MATRIX_CLOSE(t1.q_.arr_, (sign(t2.q_.w()) * sign(t1.q_.w())) * t2.q_.arr_, tol); \
    MATRIX_CLOSE(t1.t_, t2.t_, tol)

#define SO3_EQUALS(r1, r2) MATRIX_EQUALS((r1).matrix(), (r2).matrix())
#define SO3_CLOSE(r1, r2, tol) MATRIX_CLOSE((r1).matrix(), (r2).matrix(), tol)

#define SE3_EQUALS(T1, T2) MATRIX_EQUALS((T1).matrix(), (T2).matrix())
#define SE3_CLOSE(T1, T2, tol) MATRIX_CLOSE((T1).matrix(), (T2).matrix(), tol)

//  Check if m1 == m2 or m1 == -m2 (sign is ambiguous)
#define MATRIX_CLOSE_AMG_SIGN(m1, m2, tol)                                           \
    {                                                                                \
        double sign = (std::signbit(m1(0, 0)) == std::signbit(m2(0, 0))) ? 1. : -1.; \
        MATRIX_CLOSE(m1, sign*(m2), tol);                                            \
    }
