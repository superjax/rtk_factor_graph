#pragma once

#include "common/math/dquat.h"
#include "common/matrix_defs.h"

namespace mc {
namespace math {

template <typename T>
struct Jet
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    DQuat<T> x;
    typename DQuat<T>::TangentVector dx;

    static Jet Random()
    {
        Jet out;
        out.x = math::DQuat<T>::Random();
        out.dx.setRandom();
        return out;
    }

    static Jet Identity()
    {
        Jet out;
        out.x = math::DQuat<T>::identity();
        out.dx.setZero();
        return out;
    }
};

}  // namespace math
}  // namespace mc
