#pragma once

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

    ImuErrorState& operator=(const Vec9& other)
    {
        Vec9::operator=(other);
        return *this;
    }
};

struct ImuState : public Vec10
{
    static constexpr int DOF = 9;
    ImuState();
    ImuState(const ImuState& other);
    ImuState(const Vec10& other);
    Eigen::Map<Vec3> alpha;
    Eigen::Map<Vec3> beta;
    math::Quat<double> gamma;

    void setIdentity();
    void setRandom();
    ImuState& operator=(const ImuState& other);
    ImuState& operator=(const Vec10& other);
    ImuState operator+(const ImuErrorState& dstate) const;
    ImuState& operator+=(const ImuErrorState& dstate);
    ImuErrorState operator-(const ImuState& other) const;
};

}  // namespace factors
}  // namespace mc
