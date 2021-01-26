#pragma once

#include <Eigen/Core>

#include "common/ephemeris/eph.h"
#include "common/ephemeris/glonass.h"
#include "common/error.h"
#include "common/matrix_defs.h"
#include "common/out.h"
#include "common/utctime.h"

namespace mc {
namespace satellite {

struct SatelliteState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    UTCTime t = INVALID_TIME;
    Vec3 pos;
    Vec3 vel;
    Vec2 clk;
};

Error eph2Sat(const UTCTime& t, const ephemeris::KeplerianEphemeris& eph, Out<SatelliteState> sat);
Error eph2Sat(const UTCTime& t, const ephemeris::GlonassEphemeris& eph, Out<SatelliteState> sat);

Vec6 glonassOrbit(const Vec6& x, const Vec3& acc);

}  // namespace satellite
}  // namespace mc
