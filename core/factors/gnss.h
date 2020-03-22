#pragma once

#include "core/factors/base.h"

namespace mc {

namespace factors {

class Gnss : public FactorTemplate<Gnss,
                                   states::Pose,
                                   states::Velocity,
                                   states::ClockBias,
                                   states::AntennaPosition>
{
 public:
    const states::Pose& pose;
    const states::Velocity& vel;
    const states::ClockBias& clock_bias;
    const states::AntennaPosition& antenna_position;

    Gnss(const states::Pose& _pose,
         const states::Velocity& _vel,
         const states::ClockBias& _clock_bias,
         const states::AntennaPosition& _antenna_position)
        : FactorTemplate(_pose, _vel, _clock_bias, _antenna_position),
          pose(_pose),
          vel(_vel),
          clock_bias(_clock_bias),
          antenna_position(_antenna_position){};

    template <typename T>
    bool operator()() const;
};

}  // namespace factors

}  // namespace mc
