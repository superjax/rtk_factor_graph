#pragma once

#include "client/parsers/eph.h"
#include "third_party/rtklib/rtklib.h"
#include "utils/utctime.h"

namespace rtklib
{
eph_t toRtklib(const KeplerianEphemeris& eph);
gtime_t toRtklib(const UTCTime& t);

}  // namespace rtklib
