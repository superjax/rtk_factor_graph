#include "common/math/quat.h"
#include "common/matrix_defs.h"

Vec3 dont_optimize;

Vec3 rotate(const Quat<double>& R, const Vec3& v)
{
    return R.rotp(v);
}

int main()
{
    // This file is designed to be an easy-to-use scratch pad for playing with ideas.
    Quat<double> R = Quat<double>::Random();
    dont_optimize = rotate(R, Vec3::Random());
}
