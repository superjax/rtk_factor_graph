#pragma once

#include "common/matrix_defs.h"
#include "core/states/base.h"

namespace mc {

namespace states {

class Velocity final : public StateTemplate<Velocity>
{
 public:
    static constexpr int DIM = 3;
    static constexpr int ID = STATE_ID_VELOCITY;

    const double* const getData() const { return vel.data(); };
    double* const getData() { return vel.data(); };

    UTCTime t;
    Vec3 vel;
};

}  // namespace states

}  // namespace mc
