#include <gtest/gtest.h>

#include "common/satellite/azel.h"

TEST(Azel, Los2AzelStraightUp)
{
    const Vec3 surface_lla(deg2Rad(37.36964), deg2Rad(-122.0387867), 0);
    const Vec3 surface_ecef = WGS84::lla2ecef(surface_lla);

    const Vec3 sat_lla = surface_lla + 20'000'000 * Vec3::UnitZ();
    const Vec3 sat_ecef = WGS84::lla2ecef(sat_lla);

    const Vec3 los_ecef = (sat_ecef - surface_ecef).normalized();

    const AzimuthElevation az_el = los2AzEl(surface_ecef, los_ecef);

    ASSERT_NEAR(az_el.el, M_PI / 2.0, 1e-7);
}

TEST(Azel, Los2Azel)
{
    const Vec3 rec_lla(deg2Rad(40.246184), deg2Rad(-111.647769), 1387.997511);
    const Vec3 rec_ecef = WGS84::lla2ecef(rec_lla);
    const Vec3 sat_pos_ecef(-12611434.2, -13413103.98, 19062913.07);

    const Vec3 los_ecef = sat_pos_ecef - rec_ecef;
    const AzimuthElevation azel = los2AzEl(rec_ecef, los_ecef);

    EXPECT_NEAR(azel.az, -1.09260980, 1e-8);
    EXPECT_NEAR(azel.el, 1.18916781, 1e-8);
}

TEST(Azel, FromSatState)
{
    std::array<Vec3, 9> sat_pos_ecef{Vec3{-20001953.27, 8729935.619, 15051224.46},
                                     Vec3{-25574011.39, -6071212.899, -4342791.756},
                                     Vec3{8929176.251, -22924125.42, 9812860.943},
                                     Vec3{-21403708.16, -4524099.668, 14368737},
                                     Vec3{-7432731.101, -16010865.64, 20199326.85},
                                     Vec3{-17695285.31, -11996271.66, 15377378.96},
                                     Vec3{-18339125.74, 1117944.966, 19399468.97},
                                     Vec3{-6531131.647, -25187780.73, 4025755.291},
                                     Vec3{1063512.928, -15290375.53, 21733218.66}};

    const std::array<AzimuthElevation, 9> truth_azel = {
        AzimuthElevation{deg2Rad(-62), deg2Rad(6)},  AzimuthElevation{deg2Rad(-122), deg2Rad(5)},
        AzimuthElevation{deg2Rad(104), deg2Rad(38)}, AzimuthElevation{deg2Rad(-81), deg2Rad(33)},
        AzimuthElevation{deg2Rad(-14), deg2Rad(78)}, AzimuthElevation{deg2Rad(-89), deg2Rad(54)},
        AzimuthElevation{deg2Rad(-58), deg2Rad(26)}, AzimuthElevation{deg2Rad(166), deg2Rad(48)},
        AzimuthElevation{deg2Rad(40), deg2Rad(60)},
    };

    const Vec3 rec_pos_ecef(-1798904.13, -4532227.1, 4099781.95);

    for (size_t i = 0; i < sat_pos_ecef.size(); ++i)
    {
        SatelliteState sat_state;
        sat_state.pos = sat_pos_ecef[i];
        AzimuthElevation azel = getAzEl(rec_pos_ecef, sat_state);
        EXPECT_NEAR(rad2Deg(azel.az), rad2Deg(truth_azel[i].az), 1);
        EXPECT_NEAR(rad2Deg(azel.el), rad2Deg(truth_azel[i].el), 1);
    }
}
