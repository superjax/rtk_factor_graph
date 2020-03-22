#pragma once

#include "common/matrix_defs.h"
#include "core/states/base.h"

namespace mc {

namespace states {

class AntennaPosition : public StateTemplate<AntennaPosition>
{
 public:
    static constexpr int DIM = 3;
    static constexpr int ID = STATE_ID_ANTENNA_POSITION;

    double *const getData() { return p_b2g.data(); }
    const double *const getData() const { return p_b2g.data(); }
    Vec3 p_b2g;
};

}  // namespace states

}  // namespace mc
