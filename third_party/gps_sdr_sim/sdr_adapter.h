#pragma once

#include "common/ephemeris/gps.h"
#include "common/utctime.h"

#include "third_party/gps_sdr_sim/gps_sdr_sim.h"

namespace third_party {
namespace gps_sdr_sim {

gpstime_t toSdr(const mc::UTCTime& t);

ephem_t toSdr(const mc::ephemeris::GPSEphemeris& eph);

}  // namespace gps_sdr_sim
}  // namespace third_party
