#pragma once

namespace mc {
namespace core {

struct ImuBias : public Vec6
{
    auto accel() { return head<3>(); }
    auto gyro() { return tail<3>(); }
};

}  // namespace core
}  // namespace mc
