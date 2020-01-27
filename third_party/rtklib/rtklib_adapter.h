#pragma once

#include "client/parsers/eph.h"
#include "client/parsers/glonass.h"
#include "third_party/rtklib/rtklib.h"
#include "utils/utctime.h"

namespace third_party {
namespace rtklib {

eph_t toRtklib(const mc::client::parsers::KeplerianEphemeris& eph);
gtime_t toRtklib(const mc::utils::UTCTime& t);
geph_t toRtklib(const mc::client::parsers::GlonassEphemeris& eph);

}  // namespace rtklib
}  // namespace third_party
