#pragma once

#include "common/ephemeris/eph.h"
#include "common/ephemeris/glonass.h"
#include "common/utctime.h"
#include "third_party/rtklib/rtklib.h"

namespace third_party {
namespace rtklib {

eph_t toRtklib(const mc::ephemeris::KeplerianEphemeris& eph);
gtime_t toRtklib(const mc::UTCTime& t);
geph_t toRtklib(const mc::ephemeris::GlonassEphemeris& eph);

}  // namespace rtklib
}  // namespace third_party
