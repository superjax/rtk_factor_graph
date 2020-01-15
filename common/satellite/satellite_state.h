#pragma once

#include <Eigen/Core>

#include "client/parsers/eph.h"
#include "client/parsers/glonass.h"
#include "common/matrix_defs.h"
#include "utils/utctime.h"

struct SatelliteState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Vec3 pos;
    Vec3 vel;
    Vec2 clk;
};
