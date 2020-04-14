#pragma once

#include "common/math/dquat.h"
#include "common/matrix_defs.h"

namespace mc {
namespace math {

template <typename T>
struct TwoJet
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    DQuat<T> x;
    typename DQuat<T>::TangentVector dx;
    typename DQuat<T>::TangentVector d2x;

    static TwoJet Random()
    {
        TwoJet out;
        out.x = math::DQuat<T>::Random();
        out.dx.setRandom();
        out.d2x.setRandom();
        return out;
    }

    static TwoJet Identity()
    {
        TwoJet out;
        out.x = math::DQuat<T>::identity();
        out.dx.setZero();
        out.d2x.setZero();
        return out;
    }
};

}  // namespace math
}  // namespace mc
