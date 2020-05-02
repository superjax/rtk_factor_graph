#include <gtest/gtest.h>

#include "common/ephemeris/gps.h"
#include "common/satellite/satellite.h"
#include "common/test_helpers.h"

namespace mc {
namespace satellite {

using GpsEph = ephemeris::GPSEphemeris;
using GloEph = ephemeris::GlonassEphemeris;

TEST(Satellite, CreateKeplerian)
{
    Satellite<GpsEph> sat(GnssID::GPS, 0);
    EXPECT_EQ(sat.gnssId(), GnssID::GPS);
    EXPECT_EQ(sat.sat_num(), 0);
    EXPECT_EQ(sat.almanacSize(), 0);
}

TEST(Satellite, AddKeplerEph)
{
    Satellite<GpsEph> sat(GnssID::GPS, 0);

    GpsEph eph(0);
    eph.gnssID = GnssID::GPS;
    eph.sat = 0;
    eph.toe = UTCTime(101);
    sat.addEph(std::move(eph));

    EXPECT_EQ(sat.almanacSize(), 1);
}

TEST(Satellite, AlmanacIsSorted)
{
    Satellite<GpsEph> sat(GnssID::GPS, 0);

    for (int i = 0; i < 100; ++i)
    {
        GpsEph eph(0);
        eph.gnssID = GnssID::GPS;
        eph.sat = 0;
        eph.toe = UTCTime(rand() % 10000);
        sat.addEph(std::move(eph));
    }

    double prev_toe = -1;
    for (const auto& eph : sat.almanac_)
    {
        EXPECT_LT(prev_toe, eph.toe.toSec());
        prev_toe = eph.toe.toSec();
    }
}

TEST(Satellite, FindClosestEph)
{
    Satellite<GpsEph> sat(GnssID::GPS, 0);

    for (int i = 0; i < 100; ++i)
    {
        GpsEph eph(0);
        eph.gnssID = GnssID::GPS;
        eph.sat = 0;
        eph.toe = UTCTime(i * 10);
        eph.toes = i * 10;
        sat.addEph(std::move(eph));
    }

    EXPECT_EQ(sat.findClosestEphemeris(UTCTime(51.1)).toes, 50u);
    EXPECT_EQ(sat.findClosestEphemeris(UTCTime(56.1)).toes, 60u);
}

TEST(Satellite, WrongGnssIDThrow)
{
    Satellite<GpsEph> sat(GnssID::GPS, 0);
    GpsEph eph(0);
    eph.gnssID = GnssID::Glonass;
    eph.sat = 0;
    eph.toe = UTCTime(0);
#ifndef DISABLE_CHECK
    EXPECT_DIE(sat.addEph(std::move(eph)), "");
#endif
}

TEST(Satellite, WrongSatNumThrow)
{
    Satellite<GpsEph> sat(GnssID::GPS, 0);
    GpsEph eph(0);
    eph.gnssID = GnssID::GPS;
    eph.sat = 1;
    eph.toe = UTCTime(0);
#ifndef DISABLE_CHECK
    EXPECT_DIE(sat.addEph(std::move(eph)), "");
#endif
}

TEST(Satellite, InitWrongGnssID)
{
#ifndef DISABLE_CHECK
    EXPECT_DIE(Satellite<GpsEph> sat(GnssID::Glonass, 0), "");
    EXPECT_DIE(Satellite<GloEph> sat(GnssID::GPS, 0), "");
    EXPECT_DIE(Satellite<GloEph> sat(GnssID::Galileo, 0), "");
    EXPECT_DIE(Satellite<GloEph> sat(GnssID::Beidou, 0), "");
    EXPECT_DIE(Satellite<GloEph> sat(GnssID::Qzss, 0), "");
    EXPECT_DIE(Satellite<GloEph> sat(GnssID::SBAS, 0), "");
#endif
}

TEST(Satellite, ReplaceDuplicate)
{
    Satellite<GpsEph> sat(GnssID::GPS, 0);

    UTCTime saved_time;
    for (int i = 0; i < 100; ++i)
    {
        GpsEph eph(0);
        eph.gnssID = GnssID::GPS;
        eph.sat = 0;
        int random = rand() % 10000;
        eph.toe = UTCTime(random);
        eph.toes = random;
        if (i == 50)
        {
            saved_time = eph.toe;
            eph.toes = 101;
        }
        sat.addEph(eph);
    }

    const int size_before_insert = sat.almanacSize();
    GpsEph eph(0);
    eph.gnssID = GnssID::GPS;
    eph.sat = 0;
    eph.toe = saved_time;
    eph.toes = 201u;
    sat.addEph(eph);

    EXPECT_EQ(sat.almanacSize(), size_before_insert);
    EXPECT_EQ(sat.findClosestEphemeris(saved_time).toes, 201u);
}

TEST(Satellite, CreateGlonass)
{
    Satellite<GloEph> sat(GnssID::Glonass, 0);
    EXPECT_EQ(sat.gnssId(), GnssID::Glonass);
    EXPECT_EQ(sat.sat_num(), 0);
    EXPECT_EQ(sat.almanacSize(), 0);
}

TEST(Satellite, AddGlonassEph)
{
    Satellite<GloEph> sat(GnssID::Glonass, 0);

    GloEph eph;
    eph.gnssID = GnssID::Glonass;
    eph.sat = 0;
    eph.toe = UTCTime(101);
    sat.addEph(std::move(eph));

    EXPECT_EQ(sat.almanacSize(), 1);
}

TEST(Satellite, GloAlmanacIsSorted)
{
    Satellite<GloEph> sat(GnssID::Glonass, 0);

    for (int i = 0; i < 100; ++i)
    {
        GloEph eph;
        eph.gnssID = GnssID::Glonass;
        eph.sat = 0;
        eph.toe = UTCTime(rand() % 10000);
        sat.addEph(std::move(eph));
    }

    double prev_toe = -1;
    for (const auto& eph : sat.almanac_)
    {
        EXPECT_LT(prev_toe, eph.toe.toSec());
        prev_toe = eph.toe.toSec();
    }
}
TEST(Satellite, FindClosestGloEph)
{
    Satellite<GloEph> sat(GnssID::Glonass, 0);

    for (int i = 0; i < 100; ++i)
    {
        GloEph eph;
        eph.gnssID = GnssID::Glonass;
        eph.sat = 0;
        eph.toe = UTCTime(i * 10);
        eph.iode = i * 10;
        sat.addEph(std::move(eph));
    }

    EXPECT_EQ(sat.findClosestEphemeris(UTCTime(51.1)).iode, 50);
    EXPECT_EQ(sat.findClosestEphemeris(UTCTime(56.1)).iode, 60);
}

TEST(Satellite, ReplaceDuplicateGlo)
{
    Satellite<GloEph> sat(GnssID::Glonass, 0);

    UTCTime saved_time;
    for (int i = 0; i < 100; ++i)
    {
        GloEph eph;
        eph.gnssID = GnssID::Glonass;
        eph.sat = 0;
        int random = rand() % 10000;
        eph.toe = UTCTime(random);
        eph.iode = random;
        if (i == 50)
        {
            saved_time = eph.toe;
            eph.iode = 101;
        }
        sat.addEph(eph);
    }

    const int size_before_insert = sat.almanacSize();
    GloEph eph;
    eph.gnssID = GnssID::Glonass;
    eph.sat = 0;
    eph.toe = saved_time;
    eph.iode = 201u;
    sat.addEph(eph);

    EXPECT_EQ(sat.almanacSize(), size_before_insert);
    EXPECT_EQ(sat.findClosestEphemeris(saved_time).iode, 201);
}

class KeplerSat : public ::testing::Test
{
 public:
    KeplerSat() : sat(GnssID::GPS, 1) {}
    UTCTime t;
    void SetUp() override
    {
        ephemeris::GPSEphemeris eph(1);
        const int week = 86400.00 / UTCTime::SEC_IN_WEEK;
        const int tow_sec = 86400.00 - (week * UTCTime::SEC_IN_WEEK);
        t = UTCTime::fromGPS(week, tow_sec * 1000);

        int toe_week = 93600.0 / UTCTime::SEC_IN_WEEK;
        int toe_tow_sec = 93600.0 - (toe_week * UTCTime::SEC_IN_WEEK);

        eph.sat = 1;
        eph.gnssID = GnssID::GPS;
        eph.sqrta = 5153.79589081;
        eph.toe = UTCTime::fromGPS(toe_week, toe_tow_sec * 1000);
        eph.toc = eph.toe;
        eph.toes = 93600.0;
        eph.delta_n = 0.465376527657e-08;
        eph.m0 = 1.05827953357;
        eph.ecc = 0.00223578442819;
        eph.w = 2.06374037770;
        eph.cus = 0.177137553692e-05;
        eph.cuc = 0.457651913166e-05;
        eph.crs = 88.6875000000;
        eph.crc = 344.96875;
        eph.cis = -0.856816768646e-07;
        eph.cic = 0.651925802231e-07;
        eph.idot = 0.342514267094e-09;
        eph.i0 = 0.961685061380;
        eph.omega0 = 1.64046615454;
        eph.omegadot = -0.856928551657e-08;
        eph.af0 = 0.0;
        eph.af1 = 0.0;
        eph.af2 = 0.0;
        sat.addEph(eph);
    }

    Satellite<GpsEph> sat;
};

TEST_F(KeplerSat, checkPVT)
{
    const Vec3 truth_pos(-12611434.19782218519, -13413103.97797041226, 19062913.07357876760);
    const Vec3 truth_vel(266.280379332602, -2424.768347293139, -1529.762077704072);
    SatelliteState sat_state;
    const Error result = sat.getState(t, Out(sat_state));

    EXPECT_OK(result);
    MATRIX_CLOSE(truth_pos, sat_state.pos, 1e-5);
    MATRIX_CLOSE(truth_vel, sat_state.vel, 1e-5);
}

}  // namespace satellite
}  // namespace mc
