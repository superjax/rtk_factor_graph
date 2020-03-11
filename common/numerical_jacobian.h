#pragma once

#include <Eigen/Core>

namespace mc {

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

// f(Rⁿ, ...) → Rⁿ
// @returns ∂f/∂x, x ∈ Rⁿ
template <typename T, typename Fun, typename... Args>
Eigen::MatrixXd compute_jac(const T& x, const Fun& fun, const Args&... args)
{
    const double eps = 1e-8;
    Eigen::MatrixXd out;
    for (int i = 0; i < x.rows(); ++i)
    {
        const T xm = x - (T::Unit(i) * eps);
        const T xp = x + (T::Unit(i) * eps);
        const auto yp = fun(xp, args...);
        const auto ym = fun(xm, args...);

        if (out.rows() == 0 || out.cols() == 0)
        {
            out.resize(decltype(yp - yp)::RowsAtCompileTime, xp.rows());
        }

        out.col(i) = (yp - ym) / (2.0 * eps);
    }
    return out;
}

}  // namespace mc
