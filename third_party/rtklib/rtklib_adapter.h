#pragma once

#include "client/parsers/eph.h"
#include "client/parsers/glonass.h"
#include "third_party/rtklib/rtklib.h"
#include "utils/utctime.h"

namespace rtklib
{
eph_t toRtklib(const KeplerianEphemeris& eph);
gtime_t toRtklib(const UTCTime& t);
geph_t toRtklib(const GlonassEphemeris& eph);

}  // namespace rtklib
