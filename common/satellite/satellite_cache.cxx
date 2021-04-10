#include "common/satellite/satellite_cache.h"

namespace mc {
namespace satellite {

namespace {

// maximum difference in time before recomputing constants
// static constexpr double MAX_CACHE_DT = 0.0001;
// static constexpr double MAX_REC_POSITION_DELTA = 0.01;

inline double computeSagnac(const Vec3 sat_pos, const Vec3& rec_pos_ecef)
{
    return OMEGA_EARTH * (sat_pos.x() * rec_pos_ecef.y() - sat_pos.y() * rec_pos_ecef.x()) /
           C_LIGHT;
}

}  // namespace

SatelliteCache::SatelliteCache()
{
    cache_rec_pos = Vec3::Zero();
    state.t = INVALID_TIME;
}

void SatelliteCache::update(const UTCTime& t, const Vec3 rec_pos_ecef, const SatelliteBase& sat)
{
    check(t != INVALID_TIME);
    check(rec_pos_ecef.norm() > 1);

    // if (std::abs((t - state.t).toSec()) <= MAX_CACHE_DT)
    // {
    //     return;
    // }

    // if (std::abs((rec_pos_ecef - cache_rec_pos).norm()) <= MAX_REC_POSITION_DELTA)
    // {
    //     return;
    // }

    check(sat.almanacSize() > 0, "Cannot update satellite cache without almanac");

    sat.getState(t, make_out(state));
    double dt = std::numeric_limits<double>::max();
    while (abs(dt) * state.vel.norm() > 1e-3)  // get millimeter-accurate
    {
        // Compute how long it took for the signal to get here (including Sagnac Correction)
        Vec3 los_to_sat = state.pos - rec_pos_ecef;
        sagnac = computeSagnac(state.pos, rec_pos_ecef);
        double range = los_to_sat.norm() + sagnac;

        double tau = range / C_LIGHT;
        dt = tau - (t - state.t).toSec();
        sat.getState(state.t - dt, make_out(state));
    }

    satellite::computeAtmCorrection(t, rec_pos_ecef, state, make_out(atm));
}

}  // namespace satellite
}  // namespace mc
