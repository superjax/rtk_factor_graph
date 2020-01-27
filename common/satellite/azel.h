#pragma once

#include "common/math/dquat.h"
#include "common/matrix_defs.h"
#include "common/satellite/satellite_state.h"
#include "utils/wgs84.h"

namespace mc {
namespace satellite {

struct AzimuthElevation
{
    double az;
    double el;
};

void los2AzEl(const Vec3& receiver_pos_ecef, const Vec3& los_ecef, AzimuthElevation* az_el);

AzimuthElevation los2AzEl(const Vec3& receiver_pos_ecef, const Vec3& los_ecef);

AzimuthElevation getAzEl(const Vec3& receiver_pos_ecef, const SatelliteState& sat);

void azEl2Los(const Vec3& receiver_pos_ecef, const AzimuthElevation& az_el, Vec3* los_ecef);

Vec3 azEl2Los(const Vec3& receiver_pos_ecef, const AzimuthElevation& az_el);

}  // namespace satellite
}  // namespace mc
