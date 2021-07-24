#pragma once

#include <Eigen/Core>
#include <cmath>
#include <iomanip>

#include "common/math/dquat.h"
#include "gtest/gtest.h"

namespace mc {

double rel_err(double a, double b)
{
    return std::abs(a - b) / std::max({std::abs(a), std::abs(b), 1.0});
}

template <typename Derived1, typename Derived2>
std::vector<int> col_print_width(const Derived1& a, const Derived2& b)
{
    std::vector<int> widths;
    for (int col = 0; col < a.cols(); ++col)
    {
        bool zero = true;
        bool one = false;
        for (int row = 0; row < a.rows(); ++row)
        {
            if (a(row, col) != 0 || b(row, col) != 0)
            {
                zero = false;
                if (a(row, col) == 1 || a(row, col) == -1 || b(row, col) == 1 || b(row, col) == -1)
                {
                    one = true;
                }
                else
                {
                    one = false;
                    break;
                }
            }
        }
        widths.push_back(zero ? 0 : one ? 2 : 12);
    }
    return widths;
}

template <typename Derived1, typename Derived2>
std::string print_side_by_side(const Derived1& a, const Derived2& b, const double tol)
{
    std::stringstream ss;
    const auto widths = col_print_width(a, b);
    for (typename Derived1::Index row = 0; row < a.rows(); ++row)
    {
        const auto print_row_side_by_side = [&](const auto& mat) {
            for (int col = 0; col < mat.cols(); ++col)
            {
                if (col != 0)
                {
                    ss << ", ";
                }
                if (rel_err(a(row, col), b(row, col)) > tol)
                {
                    ss << "\033[1;31m" << std::setw(widths[col]) << mat(col) << "\033[0m";
                }
                else if (a(row, col) != 0)
                {
                    ss << "\033[0;34m" << std::setw(widths[col]) << mat(col) << "\033[0m";
                }
                else
                {
                    ss << std::setw(widths[col]) << mat(col);
                }
            }
        };
        print_row_side_by_side(a.row(row));
        ss << "\033[1;36m"
           << "  |  "
           << "\033[0m";
        print_row_side_by_side(b.row(row));
        ss << "\n";
    }
    return ss.str();
}

template <typename Derived1, typename Derived2>
std::string print_after_one_another(const Derived1& a, const Derived2& b, const double tol)
{
    std::stringstream ss;
    const auto widths = col_print_width(a, b);
    const auto print_matrix = [&](const auto& mat) {
        for (typename Derived1::Index row = 0; row < mat.rows(); ++row)
        {
            for (int col = 0; col < mat.cols(); ++col)
            {
                if (col != 0)
                {
                    ss << ", ";
                }
                if (rel_err(a(row, col), b(row, col)) > tol)
                {
                    ss << "\033[1;31m" << std::setw(widths[col]) << mat(row, col) << "\033[0m";
                }
                else if (a(row, col) != 0)
                {
                    ss << "\033[0;34m" << std::setw(widths[col]) << mat(row, col) << "\033[0m";
                }
                else
                {
                    ss << std::setw(widths[col]) << mat(row, col);
                }
            }
            ss << "\n";
        }
    };
    print_matrix(a);
    ss << "\n\033[1;36m";
    for (const int w : widths)
    {
        for (int i = 0; i < w; ++i)
        {
            ss << "-";
        }
        ss << "--";
    }
    ss << "\033[0m\n";
    print_matrix(b);
    return ss.str();
}

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
    bool error = false;
    double max_error = 0;
    for (typename Derived1::Index row = 0; row < a.rows(); ++row)
    {
        for (typename Derived1::Index col = 0; col < b.cols(); ++col)
        {
            const double err = rel_err(a(row, col), b(row, col));
            if (err > tol)
            {
                error = true;
                max_error = (err > max_error) ? err : max_error;
            }
        }
    }
    if (!error)
    {
        return ::testing::AssertionSuccess();
    }

    static constexpr int MAX_SIDE_BY_SIDE_COLS = 6;
    std::string printout = (a.cols() > MAX_SIDE_BY_SIDE_COLS) ? print_after_one_another(a, b, tol)
                                                              : print_side_by_side(a, b, tol);

    return ::testing::AssertionFailure()
           << a_expr << " and " << b_expr << " are not within (" << tol_expr << " )\n"
           << "max error: " << max_error << " > " << tol << "\n"
           << printout << "\n";
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
        if (std::abs(sgn * (a[i]) - b[i]) > tol)
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
        if (std::abs(sgn * (a[i]) - b[i]) > tol)
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

#define MATRIX_CLOSE(m1, m2, tol) EXPECT_PRED_FORMAT3(matrixClose, m1, m2, tol);
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

#if defined(DISABLE_ASSERT) || defined(DISABLE_CHECK)
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
