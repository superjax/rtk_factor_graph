
#include "models/imu_state.h"

namespace mc {
namespace models {

constexpr int ImuState::DOF;

ImuState::ImuState() : alpha(data()), beta(data() + 3), gamma(data() + 6) {}

ImuState::ImuState(const ImuState& other)
    : Vec10(other), alpha(data()), beta(data() + 3), gamma(data() + 6)
{
}
ImuState::ImuState(const Vec10& other)
    : Vec10(other), alpha(data()), beta(data() + 3), gamma(data() + 6)
{
}

void ImuState::setIdentity()
{
    alpha.setZero();
    beta.setZero();
    gamma = math::Quat<double>::Identity();
}

void ImuState::setRandom()
{
    alpha.setRandom();
    beta.setRandom();
    gamma = math::Quat<double>::Random();
}

ImuState ImuState::operator+(const ImuErrorState& dstate) const
{
    ImuState out;
    out.alpha = alpha + dstate.alpha;
    out.beta = beta + dstate.beta;
    out.gamma = gamma * math::Quat<double>::exp(dstate.gamma);
    return out;
}

ImuState& ImuState::operator=(const ImuState& other)
{
    Vec10::operator=(other);
    return *this;
}

ImuState& ImuState::operator=(const Vec10& other)
{
    Vec10::operator=(other);
    return *this;
}

ImuState& ImuState::operator+=(const ImuErrorState& dstate)
{
    alpha += dstate.alpha;
    beta += dstate.beta;
    gamma = gamma * math::Quat<double>::exp(dstate.gamma);
    return *this;
}

ImuErrorState::ImuErrorState() : alpha(data()), beta(data() + 3), gamma(data() + 6) {}

// x1 + exp(dx) = x2
// x2 = x1 * exp(dx)
// x1⁻¹ ⋅ x2 = exp(dx)
// log(x1⁻¹ ⋅ x2) = dx
// x2 - x1 = dx

ImuErrorState ImuState::operator-(const ImuState& other) const
{
    ImuErrorState out;
    out.alpha = alpha - other.alpha;
    out.beta = beta - other.beta;
    out.gamma = (other.gamma.inverse() * gamma).log();
    return out;
}

}  // namespace models
}  // namespace mc
