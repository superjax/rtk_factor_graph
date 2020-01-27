#include "gtest/gtest.h"

#include "client/parsers/galileo.h"
#include "third_party/rtklib/rtklib.h"

namespace mc {
namespace client {
namespace parsers {

using namespace third_party;

// clang-format off
std::array<std::array<uint32_t, 9>, 23> buffer = {{
    {0x050da0ec, 0x08907cff, 0x30083c86, 0xc84a8000, 0xaaaa8000, 0x0000002a, 0xaaaabe26, 0x99ff4000, 0x00000001},
    {0x06000000, 0x01ffffff, 0x12481e89, 0xe2484000, 0xac868000, 0x0000002a, 0xaaaa7377, 0xc83f4000, 0xbfe93aa9},
    {0x09a9ca55, 0x55555555, 0x551e0060, 0x254f8000, 0xb60f8000, 0x0000002a, 0xaaaa9517, 0x66ff4000, 0x00000001},
    {0x09a9ca55, 0x55555555, 0x55420040, 0x1d5e0000, 0xa20f8000, 0x0000002a, 0xaaaa7b85, 0x273f4000, 0xbfe93aa9},
    {0x0aa03c4f, 0xe2f98403, 0xa40000ff, 0xeb008000, 0xb4878000, 0x0000002a, 0xaaaaad73, 0x72bf4000, 0x00000001},
    {0x0aa03c4f, 0xe22b2dfe, 0x120000ff, 0xeb008000, 0xb4878000, 0x0000002a, 0xaaaa7c86, 0x49ff4000, 0xbfe93aa9},
    {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d920000, 0x0000002a, 0xaaaa88af, 0x2aff4000, 0x00000001},
    {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d924000, 0x0000002a, 0xaaaa74d6, 0xd47f4000, 0xbfe93aa9},
    {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d928000, 0x0000002a, 0xaaaa9783, 0x123f4000, 0x00000001},
    {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d92c000, 0x0000002a, 0xaaaa6bfa, 0xecbf4000, 0xbfe93aa9},
    {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d930000, 0x0000002a, 0xaaaab6f7, 0x5b7f4000, 0x00000001},
    {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d934000, 0x0000002a, 0xaaaa4a8e, 0xa5ff4000, 0xbfe93aa9},
    {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d938000, 0x0000002a, 0xaaaaa9db, 0x63bf4000, 0x00000001},
    {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d93c000, 0x0000002a, 0xaaaa55a2, 0x9d3f4000, 0xbfe93aa9},
    {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d940000, 0x0000002a, 0xaaaaafcb, 0x4cbf4000, 0x00000001},
    {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d944000, 0x0000002a, 0xaaaa53b2, 0xb23f4000, 0xbfe93aa9},
    {0x02129669, 0xae0bc9c4, 0x5dd9d3ba, 0xbdf34000, 0x80ff4000, 0x0000002a, 0xaaaaa8ce, 0xe73f4000, 0x00000001},
    {0x011291e4, 0x22f62ac5, 0x000a6328, 0xaa04c000, 0xb613c000, 0x0000002a, 0xaaaa7516, 0xd8bf4000, 0xbfe93aa9},
    {0x04129300, 0x08001b47, 0x93fff99d, 0x08000000, 0x83804000, 0x0000002a, 0xaaaabbfe, 0x02ff4000, 0x00000001},
    {0x0312bff1, 0x0f48273e, 0xe3465882, 0x703e8000, 0x88dac000, 0x0000002a, 0xaaaa6e90, 0x377f4000, 0xbfe93aa9},
    {0x06000000, 0x01ffffff, 0x12481e89, 0xe2484000, 0xacac8000, 0x0000002a, 0xaaaa8d8c, 0x387f4000, 0x00000001},
    {0x050da0ec, 0x08907cff, 0x30083c86, 0xcaea8000, 0xaaaa8000, 0x0000002a, 0xaaaa7e1f, 0x5f7f4000, 0xbfe93aa9},
    {0x00955555, 0x55555555, 0x55555555, 0x50790000, 0x8d960000, 0x0000002a, 0xaaaab2e8, 0x917f4000, 0x00000001}
}};
// clang-format on

TEST(Galileo, ParseKnown)
{
    GalileoEphemeris eph(19);
    bool finished = false;
    for (const auto& buf : buffer)
    {
        if (eph.parse(reinterpret_cast<const uint8_t*>(buf.data()), 4 * 9))
        {
            EXPECT_EQ(eph.gnssID, GnssID::Galileo);
            EXPECT_EQ(eph.sat, 19);
            finished = true;

            EXPECT_EQ(eph.toe.sec, 1573014018);  // toe 2019/11/6 4:20:18
            EXPECT_EQ(eph.toc.sec, 1573014018);  // toc 2019/11/6 4:20:18

            EXPECT_EQ(eph.toe.nsec, 0);
            EXPECT_EQ(eph.toc.nsec, 0);

            EXPECT_EQ(eph.tow, 326912u);
            EXPECT_EQ(eph.iodc, 74u);
            EXPECT_EQ(eph.iode, 74u);
            EXPECT_EQ(eph.week, 2078u);
            EXPECT_EQ(eph.toes, 274800u);
            EXPECT_EQ(eph.tocs, 274800u);

            EXPECT_NEAR(eph.af2, 0, 1e-9);
            EXPECT_NEAR(eph.af1, 3.59534e-12, 1e-17);
            EXPECT_NEAR(eph.af0, -3.04537e-06, 1e-11);
            EXPECT_NEAR(eph.m0, 0.858087, 1e-6);
            EXPECT_NEAR(eph.delta_n, 2.98155e-09, 1e-14);
            EXPECT_NEAR(eph.ecc, 7.9249e-05, 1e-9);
            EXPECT_NEAR(eph.sqrta, 5440.62, 1e-2);
            EXPECT_NEAR(eph.omega0, 2.20037, 1e-5);
            EXPECT_NEAR(eph.i0, 0.958879, 1e-6);
            EXPECT_NEAR(eph.w, 1.93694, 1e-5);
            EXPECT_NEAR(eph.omegadot, -5.46416e-09, 1e-14);
            EXPECT_NEAR(eph.idot, -6.8574284965648082e-11, 1e-25);
            EXPECT_NEAR(eph.cuc, -2.12155e-06, 1e-10);
            EXPECT_NEAR(eph.cus, 1.21035e-05, 1e-10);
            EXPECT_NEAR(eph.crc, 78, 1e-4);
            EXPECT_NEAR(eph.crs, -40.09375, 1e-4);
            EXPECT_NEAR(eph.cic, 1.49012e-08, 1e-13);
            EXPECT_NEAR(eph.cis, 5.02914e-08, 1e-13);

            EXPECT_NEAR(eph.bgd_e1_e5a, -5.820766e-09, 1e-12);
            EXPECT_NEAR(eph.bgd_e1_e5b, -6.053596e-09, 1e-12);
            EXPECT_EQ(eph.e5b_hs, 0);
            EXPECT_EQ(eph.e1b_hs, 0);
            EXPECT_EQ(eph.e5b_dvs, 0);
            EXPECT_EQ(eph.e1b_dvs, 0);
        }
    }
    EXPECT_TRUE(finished);
}

TEST(Galileo, CompareRTKLIB)
{
    uint8_t subframes[] = {
        0x2,  0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x41, 0xe4,
        0xfd, 0x0,  0x4,  0x4a, 0x47, 0x90, 0x8b, 0xd8, 0xab, 0x14, 0x0,  0x29, 0x8c, 0xa2,
        0xa8, 0x13, 0xfd, 0x0,  0x8,  0x4a, 0x59, 0xa6, 0xb8, 0x2f, 0x27, 0x11, 0x77, 0x67,
        0x4e, 0xea, 0xf7, 0xcd, 0xfd, 0x0,  0xc,  0x4a, 0xff, 0xc4, 0x3d, 0x20, 0x9c, 0xfb,
        0x8d, 0x19, 0x62, 0x9,  0xc0, 0xfa, 0xfd, 0x0,  0x10, 0x4a, 0x4c, 0x0,  0x20, 0x0,
        0x6d, 0x1e, 0x4f, 0xff, 0xe6, 0x74, 0x20, 0x0,  0xfd, 0x0,  0x14, 0x36, 0x83, 0xb0,
        0x22, 0x41, 0xf3, 0xfc, 0xc0, 0x20, 0xf2, 0x1b, 0x2b, 0xaa, 0xfd, 0x0,
    };
    rtklib::eph_t rtklib_eph;
    int rtklib_result = rtklib::decode_gal_inav(subframes, &rtklib_eph);

    GalileoEphemeris eph(19);
    bool finished = false;
    for (const auto& buf : buffer)
    {
        if (eph.parse(reinterpret_cast<const uint8_t*>(buf.data()), 4 * 9))
        {
            finished = true;
            EXPECT_EQ(eph.gnssID, GnssID::Galileo);
            EXPECT_EQ(eph.sat, 19);

            EXPECT_EQ(eph.toe.sec, rtklib_eph.toe.time + UTCTime::LEAP_SECONDS);
            EXPECT_EQ(eph.toc.sec, rtklib_eph.toc.time + UTCTime::LEAP_SECONDS);

            EXPECT_EQ(eph.toe.nsec, rtklib_eph.toe.sec * 1e9);
            EXPECT_EQ(eph.toc.nsec, rtklib_eph.toc.sec * 1e9);

            EXPECT_EQ(eph.iodc, rtklib_eph.iodc);
            EXPECT_EQ(eph.iode, rtklib_eph.iode);
            EXPECT_EQ(eph.week, rtklib_eph.week);
            EXPECT_EQ(eph.toes, rtklib_eph.toes);

            EXPECT_NEAR(eph.af2, rtklib_eph.f2, 1e-9);
            EXPECT_NEAR(eph.af1, rtklib_eph.f1, 1e-17);
            EXPECT_NEAR(eph.af0, rtklib_eph.f0, 1e-11);
            EXPECT_NEAR(eph.m0, rtklib_eph.M0, 1e-6);
            EXPECT_NEAR(eph.delta_n, rtklib_eph.deln, 1e-14);
            EXPECT_NEAR(eph.ecc, rtklib_eph.e, 1e-9);
            EXPECT_NEAR(eph.sqrta, std::sqrt(rtklib_eph.A), 1e-2);
            EXPECT_NEAR(eph.omega0, rtklib_eph.OMG0, 1e-5);
            EXPECT_NEAR(eph.i0, rtklib_eph.i0, 1e-6);
            EXPECT_NEAR(eph.w, rtklib_eph.omg, 1e-5);
            EXPECT_NEAR(eph.omegadot, rtklib_eph.OMGd, 1e-14);
            EXPECT_NEAR(eph.idot, rtklib_eph.idot, 1e-24);
            EXPECT_NEAR(eph.cuc, rtklib_eph.cuc, 1e-10);
            EXPECT_NEAR(eph.cus, rtklib_eph.cus, 1e-10);
            EXPECT_NEAR(eph.crc, rtklib_eph.crc, 1e-4);
            EXPECT_NEAR(eph.crs, rtklib_eph.crs, 1e-4);
            EXPECT_NEAR(eph.cic, rtklib_eph.cic, 1e-13);
            EXPECT_NEAR(eph.cis, rtklib_eph.cis, 1e-13);

            int svh = (eph.e5b_hs << 7) | (eph.e5b_dvs << 6) | (eph.e1b_hs << 1) | eph.e1b_dvs;
            EXPECT_EQ(svh, rtklib_eph.svh);
            EXPECT_NEAR(eph.bgd_e1_e5a, rtklib_eph.tgd[0], 1e-12);
            EXPECT_NEAR(eph.bgd_e1_e5b, rtklib_eph.tgd[1], 1e-12);
        }
    }
    EXPECT_TRUE(finished);
    EXPECT_EQ(rtklib_result, 1);
}

}  // namespace parsers
}  // namespace client
}  // namespace mc
