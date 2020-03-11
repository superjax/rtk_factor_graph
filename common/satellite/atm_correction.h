#pragma once

#include "common/error.h"
#include "common/matrix_defs.h"
#include "common/out.h"
#include "common/satellite/azel.h"
#include "common/satellite/satellite_state.h"
#include "common/utctime.h"

namespace mc {
namespace satellite {

struct AtmosphericCorrection
{
    double ionospheric_delay_m;
    double tropospheric_delay_m;
};

Error computeAtmCorrection(const UTCTime& t,
                           const Vec3& rec_pos_ecef,
                           const SatelliteState& sat_state,
                           Out<AtmosphericCorrection> atm);

double computeTroposphericDelay(const UTCTime& t, const Vec3& pos, const AzimuthElevation& azel);
double computeIonosphericDelay(const UTCTime& utc_t, const Vec3& lla, const AzimuthElevation& azel);

}  // namespace satellite
}  // namespace mc
