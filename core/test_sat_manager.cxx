#include <gtest/gtest.h>

#include "common/out.h"
#include "common/test_helpers.h"
#include "core/sat_manager.h"

namespace mc {
namespace core {

TEST(SatManager, AddGpsEphemeris)
{
    SatelliteManager sat_man;

    ephemeris::GPSEphemeris gps_eph(1);
    ephemeris::GalileoEphemeris gal_eph(1);
    ephemeris::GalileoEphemeris gal_eph2(2);
    ephemeris::GlonassEphemeris glo_eph(1);
    ephemeris::GlonassEphemeris glo_eph2(2);
    ephemeris::GlonassEphemeris glo_eph3(3);

    sat_man.ephCb(gps_eph);
    sat_man.ephCb(gal_eph);
    sat_man.ephCb(gal_eph2);
    sat_man.ephCb(glo_eph);
    sat_man.ephCb(glo_eph2);
    sat_man.ephCb(glo_eph3);

    const satellite::SatelliteBase* sat;
    EXPECT_TRUE(sat_man.getSat(GnssID::GPS, 1, Out(sat)).ok());
    EXPECT_FALSE(sat_man.getSat(GnssID::GPS, 2, Out(sat)).ok());

    EXPECT_TRUE(sat_man.getSat(GnssID::Galileo, 1, Out(sat)).ok());
    EXPECT_TRUE(sat_man.getSat(GnssID::Galileo, 2, Out(sat)).ok());
    EXPECT_FALSE(sat_man.getSat(GnssID::Galileo, 3, Out(sat)).ok());

    EXPECT_TRUE(sat_man.getSat(GnssID::Glonass, 1, Out(sat)).ok());
    EXPECT_TRUE(sat_man.getSat(GnssID::Glonass, 2, Out(sat)).ok());
    EXPECT_TRUE(sat_man.getSat(GnssID::Glonass, 3, Out(sat)).ok());
    EXPECT_FALSE(sat_man.getSat(GnssID::Glonass, 4, Out(sat)).ok());

    EXPECT_FALSE(sat_man.getSat(GnssID::Beidou, 1, Out(sat)).ok());
}

TEST(SatManager, FullBuffer)
{
    SatelliteManager sat_man;

    for (int i = 0; i < 29; i++)
    {
        ephemeris::GPSEphemeris gps_eph(i);
        sat_man.ephCb(gps_eph);
    }

    for (int i = 0; i < 29; ++i)
    {
        const satellite::SatelliteBase* sat;
        EXPECT_TRUE(sat_man.getSat(GnssID::GPS, i, Out(sat)).ok());
    }

    const satellite::SatelliteBase* sat;
    ephemeris::GPSEphemeris gps_eph(29);
    sat_man.ephCb(gps_eph);
    EXPECT_FALSE(sat_man.getSat(GnssID::GPS, 0, Out(sat)).ok());
    EXPECT_TRUE(sat_man.getSat(GnssID::GPS, 29, Out(sat)).ok());
}

TEST(SatManager, RepeatSat)
{
    SatelliteManager sat_man;

    for (int i = 0; i < 30; i++)
    {
        ephemeris::GPSEphemeris gps_eph(1);
        gps_eph.toe = i;
        sat_man.ephCb(gps_eph);
    }

    const satellite::SatelliteBase* sat;
    EXPECT_TRUE(sat_man.getSat(GnssID::GPS, 1, Out(sat)).ok());

    EXPECT_EQ(sat->almanacSize(), 30);
}

}  // namespace core
}  // namespace mc
