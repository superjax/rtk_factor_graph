#pragma once

#include <Eigen/Core>
#include "common/matrix_defs.h"

namespace mc {
namespace math {

template <typename Fun, typename X, typename... Args>
X RK4(Fun&& f, double dt, const X& x0, const Args&... args)
{
    typedef decltype(f(x0, args...)) DX;
    const DX k1 = f(x0, args...);
    const X w1 = x0 + (k1 * dt / 2.0);
    const DX k2 = f(w1, args...);
    const X w2 = x0 + (k2 * dt / 2.0);
    const DX k3 = f(w2, args...);
    const X w3 = x0 + k3 * dt;
    const DX k4 = f(w3, args...);
    return x0 + ((k1 + 2.0 * k2 + 2.0 * k3 + k4) * (dt / 6.0));
}

}  // namespace math
}  // namespace mc
