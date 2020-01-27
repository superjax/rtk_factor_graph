#pragma once

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

}
