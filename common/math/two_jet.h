#pragma once

#include "common/math/dquat.h"
#include "common/matrix_defs.h"

namespace mc {
namespace math {

template <typename T>
class TwoJet
{
 public:
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

    bool operator==(const TwoJet& other) const
    {
        return (x == other.x) && (dx == other.dx) && (d2x == other.d2x);
    }
};

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const TwoJet<T>& jet)
{
    os << "x: " << jet.x << "\ndx: " << jet.dx.transpose() << "\nd2x: " << jet.d2x.transpose();
    return os;
}

}  // namespace math
}  // namespace mc
