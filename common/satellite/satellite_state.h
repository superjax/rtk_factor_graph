#pragma once

#include <Eigen/Core>

#include "client/parsers/eph.h"
#include "client/parsers/glonass.h"
#include "common/error.h"
#include "common/matrix_defs.h"
#include "utils/utctime.h"

struct SatelliteState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Vec3 pos;
    Vec3 vel;
    Vec2 clk;
};

Error eph2Sat(const UTCTime& t, const KeplerianEphemeris& eph, SatelliteState* sat);
Error eph2Sat(const UTCTime& t, const GlonassEphemeris& eph, SatelliteState* sat);

Vec6 glonassOrbit(const Vec6& x, const Vec3& acc);
