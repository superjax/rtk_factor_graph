#include <gtest/gtest.h>

#include "common/ephemeris/gps.h"

namespace mc {
namespace ephemeris {

TEST(GPSParse, KnownData)
{
    // clang-format off
    uint32_t msgs[6][10]{
        {0x22c00e31, 0xb04aebf3, 0x2dd80a, 0x8717a3e7, 0x3ff28987, 0x2b0c8c8f, 0xaa7f16f, 0x34de804e, 0xbfe9d329, 0x93beda23},
        {0x22c00e31, 0xb04b0ccb, 0x1e67abc6, 0xb45ffedf, 0xaf3f61b, 0xb3ec995, 0xaf8bdbc5, 0xa8658185, 0x815a783f, 0x14586cdc},
        {0x22c00e31, 0xb04b2d43, 0x1182e14b, 0x9057592, 0xbf518007, 0x2843254d, 0x975fa428, 0x3506dbe1, 0x92868a6e, 0xbfffee80},
        {0x22c00e31, 0xb04b4947, 0xe40028, 0x16fcca86, 0x9a781ada, 0x8ee9581b, 0x3813faaa, 0x93a4798d, 0x80000f52, 0xbb0cf018},
        {0x22c00e31, 0xb04b6a2f, 0x13bf545b, 0xd8fd7b7, 0xa602fe6, 0xbf52c317, 0x232efafd, 0x80d3a86d, 0x83658d1c, 0x24799fe3},
        {0x22c00e31, 0xb04b8b13, 0x2dd80a, 0x8717a3e7, 0x3ff28987, 0x2b0c8c8f, 0xaa7f16f, 0x34de804e, 0xbfe9d329, 0x93beda23}};
    // clang-format on

    GPSEphemeris eph(21);
    bool finished = false;
    for (int i = 0; i < 6; i++)
    {
        if (eph.parse((uint8_t*)msgs[i], 40))
        {
            finished = true;
            EXPECT_EQ(eph.sat, 21);
            EXPECT_EQ(eph.gnssID, GnssID::GPS);
            EXPECT_EQ(eph.iode, 78);
            EXPECT_EQ(eph.iodc, 78);
            EXPECT_EQ(eph.ura, 0);
            EXPECT_EQ(eph.health, 0);
            EXPECT_EQ(eph.week, 2062);
            EXPECT_EQ(eph.code_on_L2, 1);
            EXPECT_EQ(eph.alert_flag, 0);
            EXPECT_EQ(eph.toe.sec, 1563660018);
            EXPECT_EQ(eph.toe.nsec, 0);
            EXPECT_EQ(eph.toc.sec, 1563660018);
            EXPECT_EQ(eph.toc.nsec, 0);
            EXPECT_NEAR(eph.sqrta, std::sqrt(26560606.593836), 1e-6);
            EXPECT_NEAR(eph.ecc, 0.024511, 1e-6);
            EXPECT_NEAR(eph.i0, 0.949169, 1e-6);
            EXPECT_NEAR(eph.omega0, 2.358914, 1e-6);
            EXPECT_NEAR(eph.w, -1.427803, 1e-6);
            EXPECT_NEAR(eph.w, -1.427803, 1e-6);
            EXPECT_NEAR(eph.m0, 2.311086, 1e-6);
            EXPECT_NEAR(eph.delta_n, 0.000000, 1e-6);
            EXPECT_NEAR(eph.omegadot, -0.000000, 1e-6);
            EXPECT_NEAR(eph.idot, -0.000000, 1e-6);
            EXPECT_NEAR(eph.crc, 340.968750, 1e-6);
            EXPECT_NEAR(eph.crs, -21.468750, 1e-6);
            EXPECT_NEAR(eph.cuc, -0.000001, 1e-6);
            EXPECT_NEAR(eph.cus, 0.000002, 1e-6);
            EXPECT_NEAR(eph.cic, 0.000000, 1e-6);
            EXPECT_NEAR(eph.cis, -0.000000, 1e-6);
            EXPECT_NEAR(eph.toes, 597600.000000, 1e-6);
            EXPECT_EQ(eph.fit_interval_flag, 4);
            EXPECT_NEAR(eph.af0, -0.000151, 1e-6);
            EXPECT_NEAR(eph.af1, 0.000000, 1e-6);
            EXPECT_NEAR(eph.af2, 0.000000, 1e-6);
            EXPECT_NEAR(eph.tgd, 0.000000, 1e-6);
        }
    }
    EXPECT_TRUE(finished);
}

}  // namespace ephemeris
}  // namespace mc
