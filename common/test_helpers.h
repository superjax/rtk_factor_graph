#pragma once

#include <Eigen/Core>
#include <cmath>

#include "common/math/dquat.h"

namespace mc {

template <typename Derived1, typename Derived2>
::testing::AssertionResult matrixClose(const char* a_expr,
                                       const char* b_expr,
                                       const char* tol_expr,
                                       const Eigen::MatrixBase<Derived1>& a,
                                       const Eigen::MatrixBase<Derived2>& b,
                                       const double tol)
{
    if (a.rows() != b.rows() || a.cols() != b.cols())
    {
        return ::testing::AssertionFailure()
               << a_expr << " and " << b_expr << " are not the same size \n"
               << a_expr << ": " << a.rows() << "x" << a.cols() << "\n"
               << b_expr << ": " << b.rows() << "x" << b.cols();
    }
    for (typename Derived1::Index row = 0; row < a.rows(); ++row)
    {
        for (typename Derived1::Index col = 0; col < b.cols(); ++col)
        {
            if (fabs(a(row, col) - b(row, col)) > tol)
            {
                return ::testing::AssertionFailure()
                       << "element (" << row << "," << col << ") of " << a_expr << " and " << b_expr
                       << " is not within " << tol_expr << "\n"
                       << a_expr << ": \n"
                       << a << "\n"
                       << b_expr << ": \n"
                       << b << "\n"
                       << "error: " << fabs(a(row, col) - b(row, col)) << "\n"
                       << "tol: " << tol;
            }
        }
    }
    return ::testing::AssertionSuccess();
}

::testing::AssertionResult dquatClose(const char* a_expr,
                                      const char* b_expr,
                                      const char* tol_expr,
                                      const math::DQuat<double>& a,
                                      const math::DQuat<double>& b,
                                      const double tol)
{
    double sgn = sign(a.real().w() * b.real().w());
    for (size_t i = 0; i < 8; ++i)
    {
        if (fabs(sgn * (a[i]) - b[i]) > tol)
        {
            return ::testing::AssertionFailure()
                   << "element [" << i << "]"
                   << " of " << a_expr << " and " << b_expr << " is not within " << tol_expr << "\n"
                   << a_expr << ": " << a << "\n"
                   << b_expr << ": " << b << "\n"
                   << tol_expr << ": " << tol << "\n";
        }
    }
    return ::testing::AssertionSuccess();
}

::testing::AssertionResult quatClose(const char* a_expr,
                                     const char* b_expr,
                                     const char* tol_expr,
                                     const math::Quat<double>& a,
                                     const math::Quat<double>& b,
                                     const double tol)
{
    double sgn = sign(a.w() * b.w());
    for (size_t i = 0; i < 4; ++i)
    {
        if (fabs(sgn * (a[i]) - b[i]) > tol)
        {
            return ::testing::AssertionFailure()
                   << "element [" << i << "]"
                   << " of " << a_expr << " and " << b_expr << " is not within " << tol_expr << "\n"
                   << a_expr << ": " << a << "\n"
                   << b_expr << ": " << b << "\n"
                   << tol_expr << ": " << tol << "\n";
        }
    }
    return ::testing::AssertionSuccess();
}

#define MATRIX_CLOSE(m1, m2, tol) EXPECT_PRED_FORMAT3(matrixClose, (m1), (m2), tol);
#define MAT_EQ(v1, v2) MATRIX_CLOSE((v1), (v2), 1e-8)

#define QUAT_CLOSE(q1, q2, tol) EXPECT_PRED_FORMAT3(quatClose, (q1), (q2), (tol))
#define QUAT_EQ(q1, q2) QUAT_CLOSE((q1), (q2), 1e-8)

#define DQUAT_CLOSE(Q1, Q2, tol) EXPECT_PRED_FORMAT3(dquatClose, (Q1), (Q2), (tol))
#define DQUAT_EQ(Q1, Q2) DQUAT_CLOSE((Q1), (Q2), 1e-8)

#define TRANSFORM_EQUALS(t1, t2) \
    MAT_EQ((t1).t(), (t2).t());  \
    QUAT_EQ((t1).q(), (t2).q())

#define TRANSFORM_CLOSE(t1, t2, tol)                                                               \
    {                                                                                              \
        const double sign = (std::signbit((t1).q().w()) == std::signbit((t2).q().w())) ? 1. : -1.; \
        MATRIX_CLOSE((t1).q_.arr_, sign*((t2).q_.arr_), tol);                                      \
        MATRIX_CLOSE((t1).t_, (t2).t_, tol)                                                        \
    }

#define SO3_EQUALS(r1, r2) MAT_EQ((r1).matrix(), (r2).matrix())
#define SO3_CLOSE(r1, r2, tol) MATRIX_CLOSE((r1).matrix(), (r2).matrix(), tol)

#define SE3_EQUALS(T1, T2) MAT_EQ((T1).matrix(), (T2).matrix())
#define SE3_CLOSE(T1, T2, tol) MATRIX_CLOSE((T1).matrix(), (T2).matrix(), tol)

//  Check if m1 == m2 or m1 == -m2 (sign is ambiguous)
#define MATRIX_CLOSE_AMG_SIGN(m1, m2, tol)                                           \
    {                                                                                \
        double sign = (std::signbit(m1(0, 0)) == std::signbit(m2(0, 0))) ? 1. : -1.; \
        MATRIX_CLOSE(m1, sign*(m2), tol);                                            \
    }

#define EXPECT_OK(err) EXPECT_TRUE(err.ok())
#define EXPECT_NOK(err) EXPECT_FALSE(err.ok())

#ifdef DISABLE_ASSERT
#define EXPECT_DIE(...)
#else
#define EXPECT_DIE(...) EXPECT_DEATH(__VA_ARGS__)
#endif

#ifdef DISABLE_CHECK
#define EXPECT_CHECK_FAIL(...)
#else
#define EXPECT_CHECK_FAIL(...) EXPECT_DEATH(__VA_ARGS__)
#endif

}  // namespace mc
