#include "common/logging/log_format.h"

#include <gtest/gtest.h>

#include <iostream>

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/matrix_defs.h"
#include "common/measurements/gnss_observation.h"
#include "common/measurements/imu.h"
#include "common/print.h"
#include "common/utctime.h"

namespace mc {
namespace logging {

TEST(MakeFormat, SimpleTypes)
{
    {
        const YAML::Node out = makeFormat<double>({"a"});
        EXPECT_TRUE(out["a"]["type"]);
        EXPECT_EQ(out["a"]["type"].as<int>(), 0);
    }
    {
        const YAML::Node out = makeFormat<double, float, int, uint8_t, int32_t>(
            {"double", "float", "int", "uint8_t", "int32_t"});
        EXPECT_EQ(out["double"]["type"].as<int>(), detail::get_type<double>::value);
        EXPECT_EQ(out["float"]["type"].as<int>(), detail::get_type<float>::value);
        EXPECT_EQ(out["int"]["type"].as<int>(), detail::get_type<int>::value);
        EXPECT_EQ(out["uint8_t"]["type"].as<int>(), detail::get_type<uint8_t>::value);
        EXPECT_EQ(out["int32_t"]["type"].as<int>(), detail::get_type<int32_t>::value);
    }
}

TEST(MakeFormat, MatrixTypes)
{
    {
        const YAML::Node out = makeFormat<Vec3f, Mat5>({"vec3f", "Mat5"});
        EXPECT_EQ(out["vec3f"]["type"].as<int>(), detail::get_type<float>::value);
        EXPECT_EQ(out["vec3f"]["rows"].as<int>(), 3);
        EXPECT_EQ(out["vec3f"]["cols"].as<int>(), 1);
        EXPECT_EQ(out["Mat5"]["type"].as<int>(), detail::get_type<double>::value);
        EXPECT_EQ(out["Mat5"]["rows"].as<int>(), 5);
        EXPECT_EQ(out["Mat5"]["cols"].as<int>(), 5);
    }
}

TEST(MakeFormat, CustomTypes)
{
    const YAML::Node out =
        makeFormat<UTCTime, meas::ImuSample, meas::GnssObservation>({"time", "imu", "obs"});
    EXPECT_EQ(out["time"]["sec"]["type"].as<int>(), detail::get_type<int64_t>::value);
    EXPECT_EQ(out["time"]["nsec"]["type"].as<int>(), detail::get_type<int64_t>::value);

    EXPECT_EQ(out["imu"]["t"]["sec"]["type"].as<int>(), detail::get_type<int64_t>::value);
    EXPECT_EQ(out["imu"]["t"]["nsec"]["type"].as<int>(), detail::get_type<int64_t>::value);
    EXPECT_EQ(out["imu"]["accel"]["type"].as<int>(), detail::get_type<double>::value);
    EXPECT_EQ(out["imu"]["accel"]["rows"].as<int>(), 3);
    EXPECT_EQ(out["imu"]["accel"]["cols"].as<int>(), 1);
    EXPECT_EQ(out["imu"]["gyro"]["type"].as<int>(), detail::get_type<double>::value);
    EXPECT_EQ(out["imu"]["gyro"]["rows"].as<int>(), 3);
    EXPECT_EQ(out["imu"]["gyro"]["cols"].as<int>(), 1);

    EXPECT_EQ(out["obs"]["t"]["sec"]["type"].as<int>(), detail::get_type<int64_t>::value);
    EXPECT_EQ(out["obs"]["t"]["nsec"]["type"].as<int>(), detail::get_type<int64_t>::value);
    EXPECT_EQ(out["obs"]["gnss_id"]["type"].as<int>(), detail::get_type<uint8_t>::value);
    EXPECT_EQ(out["obs"]["sat_num"]["type"].as<int>(), detail::get_type<uint8_t>::value);
    EXPECT_EQ(out["obs"]["freq"]["type"].as<int>(), detail::get_type<double>::value);
    EXPECT_EQ(out["obs"]["pseudorange"]["type"].as<int>(), detail::get_type<double>::value);
    EXPECT_EQ(out["obs"]["doppler"]["type"].as<int>(), detail::get_type<double>::value);
    EXPECT_EQ(out["obs"]["carrier_phase"]["type"].as<int>(), detail::get_type<double>::value);
}

TEST(MakeFormat, EphemerisTypes)
{
    {
        const YAML::Node out = makeFormat<ephemeris::GPSEphemeris, ephemeris::GalileoEphemeris,
                                          ephemeris::GlonassEphemeris>({"gps", "gal", "glo"});
        EXPECT_EQ(out["gps"]["toe"]["sec"]["type"].as<int>(), detail::get_type<int64_t>::value);
        EXPECT_EQ(out["gps"]["toe"]["nsec"]["type"].as<int>(), detail::get_type<int64_t>::value);
        EXPECT_EQ(out["gal"]["toe"]["sec"]["type"].as<int>(), detail::get_type<int64_t>::value);
        EXPECT_EQ(out["gal"]["toe"]["nsec"]["type"].as<int>(), detail::get_type<int64_t>::value);
        EXPECT_EQ(out["glo"]["toe"]["sec"]["type"].as<int>(), detail::get_type<int64_t>::value);
        EXPECT_EQ(out["glo"]["toe"]["nsec"]["type"].as<int>(), detail::get_type<int64_t>::value);
    }
}

}  // namespace logging
}  // namespace mc
