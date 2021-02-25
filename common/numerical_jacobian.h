#pragma once

#include <Eigen/Core>

#include "common/defs.h"
#include "common/matrix_defs.h"

namespace mc {

// Get Degrees of Freedom from Type

template <typename T, std::enable_if_t<detail::is_eigen<T>::value, int> = 0>
constexpr int dof()
{
    static_assert(T::ColsAtCompileTime == 1);
    static_assert(T::RowsAtCompileTime != Eigen::Dynamic);
    return T::RowsAtCompileTime;
}

template <typename T, std::enable_if_t<!detail::is_eigen<T>::value, int> = 0>
constexpr int dof()
{
    return T::DOF;
}

template <typename T, typename Tp>
struct has_plus
{
 private:
    template <typename U, typename Up>
    static auto test(int)
        -> decltype(std::declval<U>().operator+(std::declval<Up>()), std::true_type());

    template <typename U, typename Up>
    static std::false_type test(...);

 public:
    static constexpr bool value = std::is_same<decltype(test<T, Tp>(0)), std::true_type>::value;
};

template <typename T, typename T2 = T>
struct has_minus
{
 private:
    template <typename U, typename U2>
    static auto test(int)
        -> decltype(std::declval<U>().operator-(std::declval<U2>()), std::true_type());

    template <typename U, typename U2>
    static std::false_type test(...);

 public:
    static constexpr bool value = std::is_same<decltype(test<T, T2>(0)), std::true_type>::value;
};

template <typename T, typename Vec>
auto perturb(const T& x, const Vec& v);

template <JacobianSide side,
          typename T,
          typename Vec,
          std::enable_if_t<has_plus<T, Vec>::value, int> = 0>
auto perturb(const T& x, const Vec& v)
{
    return x + v;
}

template <JacobianSide side,
          typename T,
          typename Vec,
          std::enable_if_t<!has_plus<T, Vec>::value, int> = 0>
auto perturb(const T& x, const Vec& v)
{
    if constexpr (side == JacobianSide::LEFT)
    {
        return T::exp(v) * x;
    }
    if constexpr (side == JacobianSide::RIGHT)
    {
        return x * T::exp(v);
    }
}

template <typename T, typename T2 = T>
auto difference(const T& x2, const T2& x1);

template <
    JacobianSide side,
    typename T,
    typename T2 = T,
    std::enable_if_t<!detail::is_lie_group<T>::value && !detail::is_lie_group<T2>::value, int> = 0>
auto difference(const T& x2, const T2& x1)
{
    return x2 - x1;
}

template <
    JacobianSide side,
    typename T,
    typename T2 = T,
    std::enable_if_t<detail::is_lie_group<T>::value && detail::is_lie_group<T2>::value, int> = 0>
auto difference(const T& x2, const T2& x1) -> Eigen::Matrix<typename T::Scalar, dof<T>(), 1>
{
    if constexpr (side == JacobianSide::LEFT)
    {
        return (x2 * x1.inverse()).log();
    }
    if constexpr (side == JacobianSide::RIGHT)
    {
        return (x1.inverse() * x2).log();
    }
}

template <JacobianSide side = JacobianSide::LEFT, typename T, typename Fun>
auto compute_jac(const T& x, const Fun& fun, const double eps = 1e-8)
{
    using OutType = decltype(fun(x));
    static constexpr int out_size = dof<OutType>();
    static constexpr int in_size = dof<T>();

    Eigen::Matrix<double, out_size, in_size> out;
    typedef Eigen::Matrix<double, in_size, 1> Vec;
    for (int i = 0; i < in_size; ++i)
    {
        const T xp = perturb<side>(x, Vec::Unit(i) * eps);
        const T xm = perturb<side>(x, -Vec::Unit(i) * eps);
        const auto yp = fun(xp);
        const auto ym = fun(xm);
        out.col(i) = difference<side>(yp, ym) / (2.0 * eps);
    }
    return out;
}

// f(G, ...) → G
// @returns ∂f/∂x, x ∈ G
template <typename T, typename Fun, typename... Args>
Eigen::MatrixXd left_jac(const T& x, const Fun& fun, const Args&... args)
{
    const double eps = 1e-8;
    Eigen::MatrixXd out;
    typedef Eigen::Matrix<double, T::DOF, 1> Vec;
    for (int i = 0; i < T::DOF; ++i)
    {
        const T xp = T::exp(Vec::Unit(i) * eps) * x;
        const T xm = T::exp(-Vec::Unit(i) * eps) * x;
        const auto yp = fun(xp, args...);
        const auto ym = fun(xm, args...);

        if (out.rows() == 0 || out.cols() == 0)
        {
            out.resize(decltype(yp)::DOF, T::DOF);
        }

        out.col(i) = (yp * ym.inverse()).log() / (2.0 * eps);
    }
    return out;
}

// f(G, ...) → G
// @returns ∂f/∂x, x ∈ G
template <typename T, typename Fun, typename... Args>
Eigen::MatrixXd right_jac(const T& x, const Fun& fun, const Args&... args)
{
    const double eps = 1e-8;
    Eigen::MatrixXd out;
    typedef Eigen::Matrix<double, T::DOF, 1> Vec;
    for (int i = 0; i < T::DOF; ++i)
    {
        const T xp = x * T::exp(Vec::Unit(i) * eps);
        const T xm = x * T::exp(-Vec::Unit(i) * eps);
        const auto yp = fun(xp, args...);
        const auto ym = fun(xm, args...);

        if (out.rows() == 0 || out.cols() == 0)
        {
            out.resize(decltype(yp)::DOF, T::DOF);
        }

        out.col(i) = (ym.inverse() * yp).log() / (2.0 * eps);
    }
    return out;
}

// f(G, ...) → Rⁿ
// @returns ∂f/∂x, x ∈ G
template <typename T, typename Fun, typename... Args>
Eigen::MatrixXd left_jac2(const T& x, const Fun& fun, const Args&... args)
{
    const double eps = 1e-8;
    Eigen::MatrixXd out;
    typedef Eigen::Matrix<double, T::DOF, 1> Vec;
    for (int i = 0; i < T::DOF; ++i)
    {
        const T xp = T::exp(Vec::Unit(i) * eps) * x;
        const T xm = T::exp(-Vec::Unit(i) * eps) * x;
        const auto yp = fun(xp, args...);
        const auto ym = fun(xm, args...);

        if (out.rows() == 0 || out.cols() == 0)
        {
            out.resize(yp.rows(), T::DOF);
        }

        out.col(i) = (yp - ym) / (2.0 * eps);
    }
    return out;
}

// f(G, ...) → Rⁿ
// @returns ∂f/∂x, x ∈ G
template <typename T, typename Fun, typename... Args>
Eigen::MatrixXd right_jac2(const T& x, const Fun& fun, const Args&... args)
{
    const double eps = 1e-8;
    Eigen::MatrixXd out(0, 0);
    typedef Eigen::Matrix<double, T::DOF, 1> Vec;
    for (int i = 0; i < T::DOF; ++i)
    {
        const T xp = x * T::exp(Vec::Unit(i) * eps);
        const T xm = x * T::exp(-Vec::Unit(i) * eps);
        const Eigen::MatrixXd yp = fun(xp, args...);
        const Eigen::MatrixXd ym = fun(xm, args...);

        if (out.rows() == 0 || out.cols() == 0)
        {
            out.resize(yp.rows(), T::DOF);
        }

        out.col(i) = (yp - ym) / (2.0 * eps);
    }
    return out;
}

// f(Rⁿ, ...) → G
// @returns ∂f/∂x, x ∈ Rⁿ
template <typename T, typename Fun, typename... Args>
Eigen::MatrixXd left_jac3(const T& x, const Fun& fun, const Args&... args)
{
    const double eps = 1e-8;
    Eigen::MatrixXd out;
    for (int i = 0; i < x.rows(); ++i)
    {
        const T xp = x + T::Unit(i) * eps;
        const T xm = x - T::Unit(i) * eps;
        const auto yp = fun(xp, args...);
        const auto ym = fun(xm, args...);

        if (out.rows() == 0 || out.cols() == 0)
        {
            out.resize(decltype(yp)::DOF, xp.rows());
        }

        out.col(i) = (yp * ym.inverse()).log() / (2.0 * eps);
    }
    return out;
}
// f(Rⁿ, ...) → G
// @returns ∂f/∂x, x ∈ Rⁿ
template <typename T, typename Fun, typename... Args>
Eigen::MatrixXd right_jac3(const T& x, const Fun& fun, const Args&... args)
{
    const double eps = 1e-8;
    Eigen::MatrixXd out;
    for (int i = 0; i < x.rows(); ++i)
    {
        const T xp = x + T::Unit(i) * eps;
        const T xm = x - T::Unit(i) * eps;
        const auto yp = fun(xp, args...);
        const auto ym = fun(xm, args...);

        if (out.rows() == 0 || out.cols() == 0)
        {
            out.resize(decltype(yp)::DOF, xp.rows());
        }

        out.col(i) = (ym.inverse() * yp).log() / (2.0 * eps);
    }
    return out;
}

}  // namespace mc
