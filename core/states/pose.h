#pragma once

#include "common/math/dquat.h"
#include "common/matrix_defs.h"
#include "common/utctime.h"
#include "core/states/base.h"

namespace mc {

namespace states {

class Pose : public StateTemplate<Pose>
{
 public:
 public:
    static constexpr int ID = STATE_ID_POSE;
    static constexpr int DIM = 8;

    double* const getData() { return pose.data(); };
    const double* const getData() const { return pose.data(); };

    UTCTime t;
    math::DQuat<double> pose;
};

}  // namespace states

}  // namespace mc
