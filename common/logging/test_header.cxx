#include <gtest/gtest.h>

#include <iostream>

#include "common/ephemeris/gps.h"
#include "common/logging/header.h"
#include "common/matrix_defs.h"
#include "common/utctime.h"

namespace mc {
namespace logging {

TEST(format, json)
{
    std::cout << format(2.0, "a", 0) << std::endl;
    std::cout << format(1, "a", 2) << std::endl;
    std::cout << format(UTCTime(), "time", 0) << std::endl;
    std::cout << format(UTCTime(), "time", 4) << std::endl;
    std::cout << format(Mat5(), "double_matrix", 0) << std::endl;
    std::cout << format(Vec3f(), "float_vector", 4) << std::endl;
    std::cout << format(ephemeris::GPSEphemeris(1), "GpsEph", 25) << std::endl;
    std::cout << format(ephemeris::GalileoEphemeris(1), "GalEph", 3) << std::endl;
    std::cout << format(ephemeris::GlonassEphemeris(1), "GalEph", 3) << std::endl;
}

TEST(format, make_header)
{
    std::cout << makeHeader({"a", "b", "c"}, 2.0, ephemeris::GlonassEphemeris(1), Vec3f())
              << std::endl;
}

}  // namespace logging
}  // namespace mc
