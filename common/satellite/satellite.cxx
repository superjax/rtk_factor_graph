#include "common/satellite/satellite.h"
#include "common/defs.h"
#include "common/ephemeris/eph.h"
#include "common/print.h"
#include "common/satellite/satellite_state.h"

namespace mc {
namespace satellite {

using KepEph = ephemeris::KeplerianEphemeris;
using GloEph = mc::ephemeris::GlonassEphemeris;

template <>
Satellite<KepEph>::Satellite(uint8_t gnss_id, uint8_t sat_num) : SatelliteBase(gnss_id, sat_num)
{
    ASSERT(gnss_id != GnssID::Glonass, "Tried to initialize KeplerianEphemeris with glonass ID");
}

template <>
Satellite<GloEph>::Satellite(uint8_t gnss_id, uint8_t sat_num) : SatelliteBase(gnss_id, sat_num)
{
    ASSERT(gnss_id == GnssID::Glonass, "Tried to initialize Glonass Satellite with Kepler ID");
}

}  // namespace satellite
}  // namespace mc
