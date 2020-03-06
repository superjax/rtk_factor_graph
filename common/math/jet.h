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
    Vec3 linear_vel;
    Vec3 angular_vel;

    static Jet Random()
    {
        Jet out;
        out.x = math::DQuat<T>::Random();
        out.linear_vel.setRandom();
        out.angular_vel.setRandom();
        return out;
    }

    static Jet Identity()
    {
        Jet out;
        out.x = math::DQuat<T>::identity();
        out.linear_vel.setZero();
        out.angular_vel.setZero();
        return out;
    }
};

}  // namespace math
}  // namespace mc
