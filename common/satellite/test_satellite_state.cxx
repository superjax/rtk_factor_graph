#include <gtest/gtest.h>

#include "common/satellite/satellite_state.h"
#include "common/test_helpers.h"
#include "third_party/rtklib/rtklib.h"
#include "third_party/rtklib/rtklib_adapter.h"

namespace mc {
namespace satellite {

using namespace third_party;

class KeplerSatState : public ::testing::Test
{
 public:
    ephemeris::KeplerianEphemeris eph;
    UTCTime t;
    void SetUp() override
    {
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
    }
};

TEST_F(KeplerSatState, CheckSatPositionVelClockAgainstRTKLIB)
{
    rtklib::eph_t rtk_eph = rtklib::toRtklib(eph);

    const double dt = 1e-3;
    UTCTime t2 = t + dt;
    const Vec3 truth_pos(-12611434.19782218519, -13413103.97797041226, 19062913.07357876760);
    const Vec3 truth_vel(266.280379332602, -2424.768347293139, -1529.762077704072);

    // Rtklib doesn't compute velocities or clock rate.  Therefore, just use forward numerical
    // differentiation
    double oracle_clock, oracle_clock2;
    Vec3 oracle_pos, oracle_pos2;
    double var;
    rtklib::eph2pos(rtklib::toRtklib(t), &rtk_eph, oracle_pos.data(), &oracle_clock, &var);
    rtklib::eph2pos(rtklib::toRtklib(t2), &rtk_eph, oracle_pos2.data(), &oracle_clock2, &var);
    const Vec3 oracle_vel = (oracle_pos2 - oracle_pos) / dt;
    const double oracle_clk_rate = (oracle_clock2 - oracle_clock) / dt;

    SatelliteState sat_state;
    const Error result = eph2Sat(t, eph, Out(sat_state));

    EXPECT_OK(result);

    MATRIX_CLOSE(oracle_pos, sat_state.pos, 1e-5);
    MATRIX_CLOSE(oracle_vel, sat_state.vel, 1e-3);
    MATRIX_CLOSE(truth_pos, sat_state.pos, 1e-5);
    MATRIX_CLOSE(truth_vel, sat_state.vel, 1e-5);

    EXPECT_NEAR(sat_state.clk(0), oracle_clock, 1e-12);
    EXPECT_NEAR(sat_state.clk(1), oracle_clk_rate, 1e-12);
}

TEST_F(KeplerSatState, StaleEphemeris)
{
    SatelliteState sat_state;
    t -= 2.0;
    EXPECT_CHECK_FAIL(eph2Sat(t, eph, Out(sat_state)), "");
}

class GlonassSatState : public ::testing::Test
{
 public:
    ephemeris::GlonassEphemeris eph;
    UTCTime t;

    void SetUp() override
    {
        eph.gnssID = GnssID::Glonass;
        eph.sat = 42;
        eph.iode = 67;
        eph.slot = -7;
        eph.svh = 0;
        eph.sva = 3;
        eph.age = 0;
        eph.toe = UTCTime::fromGPS(2060, 135918000);
        eph.tof = UTCTime::fromGPS(2060, 135498000);
        eph.pos << -625159.179688, -11415025.3906, 22774500.9766;
        eph.vel << 2802.14977264, -1387.57514954, -626.088142395;
        eph.acc << 3.72529029846e-06, -1.86264514923e-06, -9.31322574615e-07;
        eph.taun = -5.34374267e-05;
        eph.gamn = 0;
        eph.dtaun = 8.38190317e-09;
    }
};

TEST_F(GlonassSatState, OrbitEquation)
{
    Vec6 x;
    x.head<3>() = eph.pos;
    x.tail<3>() = eph.vel;

    glonassOrbit(x, eph.acc);

    const rtklib::geph_t geph = rtklib::toRtklib(eph);
    Vec3 x_oracle;
    double clk;
    double var;
    rtklib::geph2pos(geph.toe, &geph, x_oracle.data(), &clk, &var);

    MAT_EQ(x_oracle, x.head<3>());
}

TEST_F(GlonassSatState, VsRTKLIB)
{
    UTCTime log_start(1561988124.550);

    // numerically differentiate for velocity
    const double dt = 1e-3;
    const rtklib::gtime_t gtime = rtklib::toRtklib(log_start);
    const rtklib::gtime_t gtime2 = rtklib::toRtklib(log_start + dt);
    const rtklib::geph_t geph = rtklib::toRtklib(eph);
    Vec3 oracle_pos, oracle_pos2;
    double oracle_clk, oracle_clk2;
    double var;
    rtklib::geph2pos(gtime, &geph, oracle_pos.data(), &oracle_clk, &var);
    rtklib::geph2pos(gtime2, &geph, oracle_pos2.data(), &oracle_clk2, &var);
    const Vec3 oracle_vel = (oracle_pos2 - oracle_pos) / dt;
    const double oracle_clk_rate = (oracle_clk2 - oracle_clk) / dt;

    SatelliteState sat_state;
    const Error result = eph2Sat(log_start, eph, Out(sat_state));

    EXPECT_OK(result);
    MATRIX_CLOSE(sat_state.pos, oracle_pos, 1e-8);
    MATRIX_CLOSE(sat_state.vel, oracle_vel, 3e-4);
    EXPECT_NEAR(sat_state.clk(0), oracle_clk, 1e-16);
    EXPECT_NEAR(sat_state.clk(1), oracle_clk_rate, 1e-16);
}

}  // namespace satellite
}  // namespace mc
