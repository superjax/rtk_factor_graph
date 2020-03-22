#pragma once

#include "core/factors/base.h"

namespace mc {

namespace factors {

class RelativeCarrierPhase : public FactorTemplate<RelativeCarrierPhase,
                                                   states::Pose,
                                                   states::Pose,
                                                   states::AntennaPosition,
                                                   states::ClockBias,
                                                   states::ClockBias,
                                                   states::GnssSwitch,
                                                   states::GnssSwitch>
{
 public:
    states::Pose& pose1;
    states::Pose& pose2;
    states::AntennaPosition& antenna_position;
    states::ClockBias& clock_bias1;
    states::ClockBias& clock_bias2;
    states::GnssSwitch& switch1;
    states::GnssSwitch& switch2;

    RelativeCarrierPhase(states::Pose& _pose1,
                         states::Pose& _pose2,
                         states::AntennaPosition& _antenna_position,
                         states::ClockBias& _clock_bias1,
                         states::ClockBias& _clock_bias2,
                         states::GnssSwitch& _switch1,
                         states::GnssSwitch& _switch2)
        : FactorTemplate(_pose1,
                         _pose2,
                         _antenna_position,
                         _clock_bias1,
                         _clock_bias2,
                         _switch1,
                         _switch2),
          pose1(_pose1),
          pose2(_pose2),
          antenna_position(_antenna_position),
          clock_bias1(_clock_bias1),
          clock_bias2(_clock_bias2),
          switch1(_switch1),
          switch2(_switch2){};

    template <typename T>
    bool operator()() const;
};

}  // namespace factors

}  // namespace mc
