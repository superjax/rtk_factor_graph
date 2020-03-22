#pragma once

#include "core/factors/base.h"

namespace mc {

namespace factors {

class GnssSwitch : public FactorTemplate<GnssSwitch, states::GnssSwitch, states::GnssSwitch>
{
 public:
    states::GnssSwitch& switch1;
    states::GnssSwitch& switch2;

    GnssSwitch(states::GnssSwitch& _switch1, states::GnssSwitch& _switch2)
        : FactorTemplate(_switch1, _switch2), switch1(_switch1), switch2(_switch2){};

    template <typename T>
    bool operator()() const;
};

}  // namespace factors

}  // namespace mc
