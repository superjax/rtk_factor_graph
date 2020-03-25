#include "common/satellite/satellite.h"
#include "common/defs.h"
#include "common/ephemeris/eph.h"
#include "common/ephemeris/galileo.h"
#include "common/ephemeris/gps.h"
#include "common/print.h"
#include "common/satellite/satellite_state.h"

namespace mc {
namespace satellite {

using GloEph = ephemeris::GlonassEphemeris;
using GpsEph = ephemeris::GPSEphemeris;
using GalEph = ephemeris::GalileoEphemeris;

template <>
Satellite<GpsEph>::Satellite(uint8_t gnss_id, uint8_t sat_num) : SatelliteBase(gnss_id, sat_num)
{
    check(gnss_id == GnssID::GPS, "Tried to initialize GPS Ephemeris with wrong ID");
}

template <>
Satellite<GalEph>::Satellite(uint8_t gnss_id, uint8_t sat_num) : SatelliteBase(gnss_id, sat_num)
{
    check(gnss_id == GnssID::Galileo, "Tried to initialize Galileo Ephemeris with wrong ID");
}

template <>
Satellite<GloEph>::Satellite(uint8_t gnss_id, uint8_t sat_num) : SatelliteBase(gnss_id, sat_num)
{
    check(gnss_id == GnssID::Glonass, "Tried to initialize Glonass Satellite with Kepler ID");
}

}  // namespace satellite
}  // namespace mc
