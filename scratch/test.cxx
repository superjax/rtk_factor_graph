#include "common/math/quat.h"
#include "common/matrix_defs.h"

namespace mc {
namespace scratch {

Vec3 rotate(const math::Quat<double>& R, const Vec3& v)
{
    return R.rotp(v);
}

}  // namespace scratch
}  // namespace mc

mc::Vec3 dont_optimize;

int main()
{
    // This file is designed to be an easy-to-use scratch pad for playing with ideas.
    mc::math::Quat<double> R = mc::math::Quat<double>::Random();
    dont_optimize = mc::scratch::rotate(R, mc::Vec3::Random());
}
