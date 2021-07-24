#pragma once

#include <Eigen/Core>

#include "common/defs.h"
#include "common/has_plus.h"
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

template <JacobianSide side, typename T, typename V, typename = void>
struct PerturbHelper;

template <JacobianSide side, typename T, typename V, typename = void>
struct DiffHelper;

template <JacobianSide side, typename T, typename T2>
struct PerturbHelper<side, T, T2, std::enable_if_t<detail::is_lie_group<T>::value>>
{
    static auto perturb(const T& x, const T2& v)
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
};

template <JacobianSide side, typename T, typename T2>
struct DiffHelper<side, T, T2, std::enable_if_t<detail::is_lie_group<T>::value>>
{
    static auto difference(const T& x2, const T& x1)
        -> Eigen::Matrix<typename T::Scalar, dof<T>(), 1>
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
};

template <JacobianSide side, typename T, typename T2>
struct PerturbHelper<side,
                     T,
                     T2,
                     std::enable_if_t<!detail::is_lie_group<T>::value && has_plus<T, T2>::value>>
{
    static auto perturb(const T& x2, const T2& dx) { return x2 + dx; }
};

template <JacobianSide side, typename T, typename T2>
struct DiffHelper<side,
                  T,
                  T2,
                  std::enable_if_t<!detail::is_lie_group<T>::value && has_minus<T, T2>::value>>
{
    static auto difference(const T& x2, const T2& x1)
        -> Eigen::Matrix<typename T::Scalar, dof<T>(), 1>
    {
        return x2 - x1;
    }
};

template <JacobianSide side, typename T, typename V>
auto perturb(const T& x, const V& dx)
{
    return PerturbHelper<side, T, V>::perturb(x, dx);
}

template <JacobianSide side, typename T, typename V>
auto difference(const T& x2, const V& x1)
{
    return DiffHelper<side, T, V>::difference(x2, x1);
}

template <JacobianSide side = JacobianSide::LEFT, typename T, typename Fun>
auto compute_jac(const T& x, const Fun& fun, const double eps = 1e-8)
{
    using OutType = decltype(fun(x));
    static constexpr int out_size = dof<OutType>();
    static constexpr int in_size = dof<T>();

    Eigen::Matrix<double, out_size, in_size> out;
    using Vec = Eigen::Matrix<double, in_size, 1>;
    for (int i = 0; i < in_size; ++i)
    {
        // It would be better to make dx_p and dx_m expression templates to avoid the temporary,
        // but it makes the meta-programming really difficult to match `perturb` in many cases,
        // since the expression type is a complicated template expression and partial template
        // specialization is not allowed by the standard.
        const Vec dx_p = Vec::Unit(i) * eps;
        const Vec dx_m = -Vec::Unit(i) * eps;
        const T xp = PerturbHelper<side, T, Vec>::perturb(x, dx_p);
        const T xm = PerturbHelper<side, T, Vec>::perturb(x, dx_m);
        const auto yp = fun(xp);
        const auto ym = fun(xm);
        out.col(i) = DiffHelper<side, OutType, OutType>::difference(yp, ym) / (2.0 * eps);
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
