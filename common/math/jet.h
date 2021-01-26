#pragma once

#include "common/math/dquat.h"
#include "common/matrix_defs.h"

namespace mc {
namespace math {

template <typename T>
class Jet
{
 public:
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

    bool operator==(const Jet& other) const { return (x == other.x) && (dx == other.dx); }
};

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const Jet<T>& jet)
{
    os << "x: " << jet.x << "\ndx: " << jet.dx.transpose();
    return os;
}

}  // namespace math
}  // namespace mc
