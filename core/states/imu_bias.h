#pragma once

#include "common/matrix_defs.h"
#include "core/states/base.h"

namespace mc {

namespace states {

class ImuBias : public StateTemplate<ImuBias>
{
 public:
    static constexpr int DIM = 6;
    static constexpr int ID = STATE_ID_IMU_BIAS;

    double* const getData() { return bias.data(); };
    const double* const getData() const { return bias.data(); };

    Vec6 bias;
};

}  // namespace states

}  // namespace mc
