#pragma once

#include "core/factors/base.h"

namespace mc {

namespace factors {

class Imu : public FactorTemplate<Imu,
                                  states::Pose,
                                  states::Pose,
                                  states::Velocity,
                                  states::Velocity,
                                  states::ImuBias>
{
 public:
    states::Pose& pose1;
    states::Pose& pose2;
    states::Velocity& vel1;
    states::Velocity& vel2;
    states::ImuBias& imu_bias;

    Imu(states::Pose& _pose1,
        states::Pose& _pose2,
        states::Velocity& _vel1,
        states::Velocity& _vel2,
        states::ImuBias& _bias)
        : FactorTemplate(_pose1, _pose2, _vel1, _vel2, _bias),
          pose1(_pose1),
          pose2(_pose2),
          vel1(_vel1),
          vel2(_vel2),
          imu_bias(_bias){};

    bool operator()() const;
};

}  // namespace factors

}  // namespace mc
