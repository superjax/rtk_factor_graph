#include <gtest/gtest.h>

#include "common/defs.h"
#include "common/matrix_defs.h"
#include "common/out.h"
#include "common/satellite/atm_correction.h"
#include "common/satellite/azel.h"
#include "common/test_helpers.h"
#include "utils/wgs84.h"

#include "third_party/rtklib/rtklib.h"
#include "third_party/rtklib/rtklib_adapter.h"

namespace mc {
namespace satellite {

namespace rtklib = third_party::rtklib;

class TestAtmosphericCorrection : public ::testing::Test
{
 public:
    SatelliteState sat_state;
    UTCTime t;
    void SetUp() override
    {
        ephemeris::KeplerianEphemeris eph;
        const int week = 86400.00 / UTCTime::SEC_IN_WEEK;
        const int tow_sec = 86400.00 - (week * UTCTime::SEC_IN_WEEK);
        t = UTCTime::fromGPS(week, tow_sec * 1000);

        const int toe_week = 93600.0 / UTCTime::SEC_IN_WEEK;
        const int toe_tow_sec = 93600.0 - (toe_week * UTCTime::SEC_IN_WEEK);

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

        const Error result = eph2Sat(t, eph, Out(sat_state));
        EXPECT_TRUE(result.ok());
    }
};

TEST_F(TestAtmosphericCorrection, Ionosphere)
{
    const Vec3 provo_lla{deg2Rad(40.246184), deg2Rad(-111.647769), 1387.997511};
    const Vec3 provo_ecef = utils::WGS84::lla2ecef(provo_lla);

    const rtklib::gtime_t gtime = rtklib::toRtklib(t);

    const AzimuthElevation azel = getAzEl(provo_ecef, sat_state);

    const double rtklib_azel[2] = {azel.az, azel.el};
    const double ion_model[] = {0.1118E-07, -0.7451E-08, -0.5961E-07, 0.1192E-06,
                                0.1167E+06, -0.2294E+06, -0.1311E+06, 0.1049E+07};
    const double rtklib_ion_delay =
        rtklib::ionmodel(gtime, ion_model, provo_lla.data(), rtklib_azel);

    const double ion_delay = computeIonosphericDelay(t, provo_lla, azel);

    EXPECT_NEAR(rtklib_ion_delay, ion_delay, 1e-8);
}

TEST_F(TestAtmosphericCorrection, Troposphere)
{
    const Vec3 provo_lla{deg2Rad(40.246184), deg2Rad(-111.647769), 1387.997511};
    const Vec3 provo_ecef = utils::WGS84::lla2ecef(provo_lla);

    const rtklib::gtime_t gtime = rtklib::toRtklib(t);

    const AzimuthElevation azel = getAzEl(provo_ecef, sat_state);

    const double rtklib_azel[2] = {azel.az, azel.el};
    static const double humidity = 0.7;
    const double rtklib_trop_delay =
        rtklib::tropmodel(gtime, provo_lla.data(), rtklib_azel, humidity);

    const double trop_delay = computeTroposphericDelay(t, provo_lla, azel);

    EXPECT_NEAR(rtklib_trop_delay, trop_delay, 1e-8);
}

TEST_F(TestAtmosphericCorrection, ComputeAtmCorrection)
{
    const Vec3 provo_lla{deg2Rad(40.246184), deg2Rad(-111.647769), 1387.997511};
    const Vec3 provo_ecef = utils::WGS84::lla2ecef(provo_lla);

    const rtklib::gtime_t gtime = rtklib::toRtklib(t);

    const AzimuthElevation azel = getAzEl(provo_ecef, sat_state);

    const double rtklib_azel[2] = {azel.az, azel.el};
    static const double humidity = 0.7;
    const double rtklib_trop_delay =
        rtklib::tropmodel(gtime, provo_lla.data(), rtklib_azel, humidity);
    const double ion_model[] = {0.1118E-07, -0.7451E-08, -0.5961E-07, 0.1192E-06,
                                0.1167E+06, -0.2294E+06, -0.1311E+06, 0.1049E+07};
    const double rtklib_ion_delay =
        rtklib::ionmodel(gtime, ion_model, provo_lla.data(), rtklib_azel);

    AtmosphericCorrection correction;
    const Error result = computeAtmCorrection(t, provo_ecef, sat_state, Out(correction));

    EXPECT_OK(result);
    EXPECT_NEAR(correction.tropospheric_delay_m, rtklib_trop_delay, 1e-8);
    EXPECT_NEAR(correction.ionospheric_delay_m, rtklib_ion_delay, 1e-8);
}

}  // namespace satellite
}  // namespace mc
