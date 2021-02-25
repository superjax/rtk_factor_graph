#pragma once

#include "common/error.h"
#include "common/matrix_defs.h"
#include "common/satellite/atm_correction.h"
#include "common/satellite/satellite.h"
#include "common/satellite/satellite_state.h"
#include "common/utctime.h"

namespace mc {
namespace satellite {

// This holds semi-constant values between runs of an estimator.  Even if updates are required, they
// should be relatively quick since these values are computed using iterative algorithms that should
// be already warm started.
class SatelliteCache
{
 public:
    SatelliteCache();
    Vec3 cache_rec_pos;  // reference position that this cache was computed
    SatelliteState state;
    double sagnac;
    AtmosphericCorrection atm;

    void update(const UTCTime& t, const Vec3 rec_pos_ecef, const SatelliteBase& sat);
};

}  // namespace satellite
}  // namespace mc
