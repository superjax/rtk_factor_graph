#pragma once

#include "common/math/dquat.h"
#include "common/matrix_defs.h"
#include "common/utctime.h"

namespace mc {
namespace core {

struct Node
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int MAX_SAT = 30;

    UTCTime t;
    math::DQuat<double> T_n2b;
    Vec3 vel;
    Vec2 clk;
    double sw[MAX_SAT];
};

}  // namespace core
}  // namespace mc
