#include <gtest/gtest.h>

#include "common/satellite/satellite_state.h"
#include "common/test_helpers.h"
#include "third_party/rtklib/rtklib.h"
#include "third_party/rtklib/rtklib_adapter.h"

class TestSatState : public ::testing::Test
{
 public:
    KeplerianEphemeris eph;
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

TEST_F(TestSatState, CheckSatPositionVelClockAgainstRTKLIB)
{
    rtklib::eph_t rtk_eph = rtklib::toRtklib(eph);

    double dt = 1e-3;
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
    const Error result = eph2Sat(t, eph, &sat_state);

    EXPECT_OK(result);

    MATRIX_CLOSE(oracle_pos, sat_state.pos, 1e-5);
    MATRIX_CLOSE(oracle_vel, sat_state.vel, 1e-3);
    MATRIX_CLOSE(truth_pos, sat_state.pos, 1e-5);
    MATRIX_CLOSE(truth_vel, sat_state.vel, 1e-5);

    EXPECT_NEAR(sat_state.clk(0), oracle_clock, 1e-12);
    EXPECT_NEAR(sat_state.clk(1), oracle_clk_rate, 1e-12);
}

TEST_F(TestSatState, StaleEphemeris)
{
    SatelliteState sat_state;
    t -= 2.0;
    const Error result = eph2Sat(t, eph, &sat_state);

    EXPECT_NOK(result);
}
