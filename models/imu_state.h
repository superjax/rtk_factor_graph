#pragma once

#include "common/check.h"
#include "common/math/dquat.h"
#include "common/matrix_defs.h"

namespace mc {
namespace models {

struct ImuErrorState : public Vec9
{
    ImuErrorState();

    template <typename Derived>
    ImuErrorState(const Eigen::MatrixBase<Derived>& other)
        : Vec9(other), alpha(data()), beta(data() + 3), gamma(data() + 6)
    {
        static_assert(Eigen::MatrixBase<Derived>::RowsAtCompileTime == 9);
    }

    Eigen::Map<Vec3> alpha;
    Eigen::Map<Vec3> beta;
    Eigen::Map<Vec3> gamma;

    template <typename Derived>
    ImuErrorState& operator=(const Eigen::MatrixBase<Derived>& other)
    {
        static_assert(Derived::RowsAtCompileTime == 9);
        Vec9::operator=(other);
        return *this;
    }
};

struct ImuState
{
    Vec3 alpha;
    Vec3 beta;
    math::Quat<double> gamma;

    static constexpr int DOF = 9;
    ImuState() = default;
    ImuState(const ImuState& other);

    template <typename Derived>
    ImuState(const Eigen::MatrixBase<Derived>& other)
    {
        static_assert(Derived::RowsAtCompileTime == 10);
        static_assert(Derived::ColsAtCompileTime == 1);

        alpha = other.template head<3>();
        beta = other.template segment<3>(3);
        gamma = other.template tail<4>();
    }

    double operator()(int i)
    {
        check(i >= 0, "Tried to index ImuState with i<0");
        check(i < 10, "Tried to index ImuState with i>=10");
        return i < 3 ? alpha(i) : i < 6 ? beta(i - 3) : gamma.arr_(i - 6);
    }

    void setIdentity();
    void setRandom();
    ImuState& operator=(const ImuState& other);

    template <typename Derived>
    ImuState& operator=(const Eigen::MatrixBase<Derived>& other)
    {
        static_assert(Derived::RowsAtCompileTime == 10);
        static_assert(Derived::ColsAtCompileTime == 1);

        alpha = other.template head<3>();
        beta = other.template segment<3>(3);
        gamma = other.template tail<4>();
        return *this;
    }

    ImuState operator+(const ImuErrorState& dstate) const;
    ImuState& operator+=(const ImuErrorState& dstate);
    ImuErrorState operator-(const ImuState& other) const;
};

}  // namespace models
}  // namespace mc
