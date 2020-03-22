#pragma once

#include "core/states/base.h"

namespace mc {

namespace states {

class GnssSwitch : public StateTemplate<GnssSwitch>
{
 public:
    static constexpr int DIM = 1;
    static constexpr int ID = STATE_ID_GNSS_SWITCH;

    double *const getData() { return &sw; };
    const double *const getData() const { return &sw; };
    double sw;
};

}  // namespace states
}  // namespace mc
