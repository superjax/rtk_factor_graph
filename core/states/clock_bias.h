#pragma once

#include "common/matrix_defs.h"
#include "core/states/base.h"

namespace mc {

namespace states {

class ClockBias : public StateTemplate<ClockBias>
{
 public:
    static constexpr int DIM = 2;
    static constexpr int ID = STATE_ID_CLOCK_BIAS;

    double *const getData() { return clk.data(); };
    const double *const getData() const { return clk.data(); };
    Vec2 clk;
};

}  // namespace states

}  // namespace mc
