#pragma once

namespace mc {
namespace core {

struct ImuBias : public Vec6
{
    auto accel() { return head<3>(); }
    auto gyro() { return tail<3>(); }

    template <typename Derived>
    ImuBias& operator=(const Derived& other)
    {
        Vec6::operator=(other);
        return *this;
    }

    template <typename Derived>
    ImuBias(const Derived& other) : Vec6(other)
    {
    }
    
    ImuBias() = default;
};

}  // namespace core
}  // namespace mc
