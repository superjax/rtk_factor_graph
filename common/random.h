#pragma once

#include <Eigen/Core>
#include <random>

namespace mc {

// Returns an Eigen matrix of type Derived with every element selected from a normal distribution
// with mean 0 and standard deviation 1.0
template <typename Derived>
Derived randomNormal()
{
    static std::normal_distribution<typename Derived::Scalar> N(0, 1.0);
    static std::default_random_engine g;
    Derived out;
    for (int i = 0; i < Derived::RowsAtCompileTime; i++)
    {
        for (int j = 0; j < Derived::ColsAtCompileTime; j++)
        {
            out(i, j) = N(g);
        }
    }
    return out;
}

template <typename Derived>
Derived randomUniformat(double lower_bound, double upper_bound)
{
    static std::uniform_real_distribution<double> U(0, 1.0);
    static std::default_random_engine g;
    Derived out;
    const int width = upper_bound - lower_bound;
    for (int i = 0; i < Derived::RowsAtCompileTime; i++)
    {
        for (int j = 0; j < Derived::ColsAtCompileTime; j++)
        {
            out(i, j) = width * U(g) + lower_bound;
        }
    }
    return out;
}

inline Eigen::MatrixXd randomNormal(int rows, int cols = 1)
{
    static std::normal_distribution<double> N(0, 1.0);
    static std::default_random_engine g;
    Eigen::MatrixXd out(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            out(i, j) = N(g);
        }
    }
    return out;
}

inline Eigen::MatrixXd randomUniform(double lower_bound, double upper_bound, int rows, int cols = 1)
{
    static std::uniform_real_distribution<double> U(0, 1.0);
    static std::default_random_engine g;
    Eigen::MatrixXd out(rows, cols);
    const int width = upper_bound - lower_bound;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            out(i, j) = width * U(g) + lower_bound;
        }
    }
    return out;
}

}  // namespace mc
