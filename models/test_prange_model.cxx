#include <gtest/gtest.h>
#include "common/defs.h"
#include "common/ephemeris/gps.h"
#include "common/measurements/gnss_observation.h"
#include "common/numerical_jacobian.h"
#include "common/satellite/satellite.h"
#include "common/test_helpers.h"
#include "models/prange_model.h"
#include "utils/wgs84.h"

#include "third_party/gps_sdr_sim/gps_sdr_sim.h"
#include "third_party/gps_sdr_sim/sdr_adapter.h"

namespace mc {
namespace models {

using namespace third_party;

using DQ = math::DQuat<double>;

class TestPrange : public ::testing::Test
{
 public:
    TestPrange() : sat(GnssID::GPS, 1), eph(1) {}

    void SetUp() override
    {
        const int week = 86400.00 / UTCTime::SEC_IN_WEEK;
        const int tow_sec = 86400.00 - (week * UTCTime::SEC_IN_WEEK);
        t = UTCTime::fromGPS(week, tow_sec * 1000);

        const Vec3 provo_lla{deg2Rad(40.246184), deg2Rad(-111.647769), 1387.997511};
        provo_ecef = utils::WGS84::lla2ecef(provo_lla);
        const Vec3 rec_vel(1, 2, 3);

        // int toe_week = 93600.0 / UTCTime::SEC_IN_WEEK;
        // int toe_tow_sec = 93600.0 - (toe_week * UTCTime::SEC_IN_WEEK);

        eph.sat = 1;
        eph.gnssID = GnssID::GPS;
        eph.sqrta = 5153.79589081;
        eph.toe = t - 120.0;
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

        T_n2b = DQ::Random();
        p_b2g = Vec3::Random();
        vel_b = Vec3::Random();
        clk = Vec2::Random();
        T_e2r = utils::WGS84::dq_ecef2ned(provo_ecef);
        sw = 0.3521;
        T_r2n = DQ(math::Quatd::from_euler(0, 0, 0.234), Vec3::Random());

        obs.t = t;
        obs.pseudorange = 22080611.16819984;
        obs.doppler = 592.7546427777615;
    }

    UTCTime t;
    satellite::Satellite<ephemeris::GPSEphemeris> sat;
    ephemeris::GPSEphemeris eph;
    Vec3 provo_ecef;
    meas::GnssObservation obs;

    DQ T_n2b;
    Vec3 vel_b;
    Vec2 clk;
    DQ T_e2r;
    DQ T_r2n;
    Vec3 p_b2g;
    double sw;
};

TEST_F(TestPrange, VsSdrSim)
{
    T_r2n = DQ::identity();
    p_b2g.setZero();
    vel_b.setZero();
    T_n2b = DQ::identity();
    clk.setZero();
    sw = 1.0;

    gps_sdr_sim::ionoutc_t ion = {true,       true,       0.1118E-07,  -0.7451E-08, -0.5961E-07,
                                  0.1192E-06, 0.1167E+06, -0.2294E+06, -0.1311E+06, 0.1049E+07};

    satellite::SatelliteState sat_state;
    sat.getState(t, Out(sat_state));
    gps_sdr_sim::range_t sdr_range;
    gps_sdr_sim::computeRange(&sdr_range, sat_state.pos.data(), sat_state.vel.data(),
                              sat_state.clk.data(), &ion, gps_sdr_sim::toSdr(t), provo_ecef.data());

    obs.t = t;
    obs.pseudorange = sdr_range.range;
    obs.doppler = sdr_range.rate;

    PseudorangeModel f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);

    Vec3 residuals;

    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};

    EXPECT_TRUE(f.Evaluate(parameters, residuals.data(), nullptr));

    EXPECT_LT(residuals[0], 4.0);  // I believe that we are better than SDR, since we don't
                                   // approximate satellite motions and include sagnac
    EXPECT_LT(residuals[1], 0.1);  // same here
    EXPECT_DOUBLE_EQ(residuals[2], 0.0);
}

TEST_F(TestPrange, Regression)
{
    T_r2n = DQ::identity();
    p_b2g.setZero();
    vel_b.setZero();
    T_n2b = DQ::identity();
    clk.setZero();
    sw = 1.0;

    PseudorangeModel f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);

    Vec3 residuals;
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};

    EXPECT_TRUE(f.Evaluate(parameters, residuals.data(), nullptr));

    EXPECT_DOUBLE_EQ(residuals[0], 0.0);
    EXPECT_DOUBLE_EQ(residuals[1], 0.0);
    EXPECT_DOUBLE_EQ(residuals[2], 0.0);
}

TEST_F(TestPrange, PoseJacobian)
{
    T_n2b = DQ::Random();
    p_b2g = Vec3::Random();
    vel_b = Vec3::Random();
    clk = Vec2::Random();
    sw = 0.3521;

    PseudorangeModel f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);
    const auto func = [this, &f](const Vec8& _T_n2b) {
        const double* parameters[] = {_T_n2b.data(), vel_b.data(), clk.data(),
                                      T_r2n.data(),  &sw,          p_b2g.data()};
        Vec3 residuals;
        f.Evaluate(parameters, residuals.data(), nullptr);
        return residuals;
    };

    MatRM38 jac;
    double* jacobians[] = {jac.data(), 0, 0, 0, 0, 0};
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};
    Vec3 residuals;

    f.Evaluate(parameters, residuals.data(), jacobians);

    MatRM38 num_jac = compute_jac(Vec8(T_n2b.arr_), func, 1e-4);

    // doppler jacobians are approximate
    // (we are ignoring several coriolis terms that make it to the mm/s range)
    MATRIX_CLOSE(num_jac * T_n2b.dParamDGen(), jac * T_n2b.dParamDGen(), 1e-3);
}

TEST_F(TestPrange, VelJacobian)
{
    T_n2b = DQ::Random();
    p_b2g = Vec3::Random();
    vel_b = Vec3::Random();
    clk = Vec2::Random();
    sw = 0.3521;

    PseudorangeModel f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);
    const auto func = [this, &f](const Vec3& _vel_b) {
        const double* parameters[] = {T_n2b.data(), _vel_b.data(), clk.data(),
                                      T_r2n.data(), &sw,           p_b2g.data()};
        Vec3 residuals;
        f.Evaluate(parameters, residuals.data(), nullptr);
        return residuals;
    };

    MatRM3 jac;
    double* jacobians[] = {0, jac.data(), 0, 0, 0, 0};
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};
    Vec3 residuals;
    f.Evaluate(parameters, residuals.data(), jacobians);

    Mat3 num_jac = compute_jac(vel_b, func);

    MATRIX_CLOSE(num_jac, jac, 1e-5);
}

TEST_F(TestPrange, ClkJacobian)
{
    T_n2b = DQ::Random();
    p_b2g = Vec3::Random();
    vel_b = Vec3::Random();
    clk = Vec2::Random();
    sw = 0.3521;

    PseudorangeModel f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);
    const auto func = [this, &f](const Vec2& _clk) {
        const double* parameters[] = {T_n2b.data(), vel_b.data(), _clk.data(),
                                      T_r2n.data(), &sw,          p_b2g.data()};
        Vec3 residuals;
        f.Evaluate(parameters, residuals.data(), nullptr);
        return residuals;
    };

    MatRM32 jac;
    double* jacobians[] = {0, 0, jac.data(), 0, 0, 0};
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};
    Vec3 residuals;
    f.Evaluate(parameters, residuals.data(), jacobians);

    MatRM32 num_jac = compute_jac(clk, func, 1e-5);

    MATRIX_CLOSE(num_jac, jac, 1e-4);
}

TEST_F(TestPrange, Tr2nJacobian)
{
    PseudorangeModel f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);
    const auto func = [this, &f](const Vec8& _T_r2n) {
        const double* parameters[] = {T_n2b.data(),  vel_b.data(), clk.data(),
                                      _T_r2n.data(), &sw,          p_b2g.data()};
        Vec3 residuals;
        f.Evaluate(parameters, residuals.data(), nullptr);
        return residuals;
    };

    MatRM38 jac;
    double* jacobians[] = {0, 0, 0, jac.data(), 0, 0};
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};
    Vec3 residuals;

    f.Evaluate(parameters, residuals.data(), jacobians);

    MatRM38 num_jac = compute_jac(Vec8(T_r2n.arr_), func, 1e-5);

    MATRIX_CLOSE(num_jac * T_r2n.dParamDGen(), jac * T_r2n.dParamDGen(), 1e-4);
}

TEST_F(TestPrange, SwJacobian)
{
    T_n2b = DQ::Random();
    p_b2g = Vec3::Random();
    vel_b = Vec3::Random();
    clk = Vec2::Random();
    sw = 0.3521;

    PseudorangeModel f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);
    const auto func = [this, &f](const Vec1& _sw) {
        const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                      T_r2n.data(), _sw.data(),   p_b2g.data()};
        Vec3 residuals;
        f.Evaluate(parameters, residuals.data(), nullptr);
        return residuals;
    };

    Vec3 jac;
    double* jacobians[] = {0, 0, 0, 0, jac.data(), 0};
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};
    Vec3 residuals;
    f.Evaluate(parameters, residuals.data(), jacobians);

    Vec3 num_jac = compute_jac(Vec1(sw), func);

    MATRIX_CLOSE(num_jac, jac, 1e-8);
}

TEST_F(TestPrange, LeverArmJacobian)
{
    PseudorangeModel f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);
    const auto func = [this, &f](const Vec3& _p_b2g) {
        const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                      T_r2n.data(), &sw,          _p_b2g.data()};
        Vec3 residuals;
        f.Evaluate(parameters, residuals.data(), nullptr);
        return residuals;
    };

    MatRM3 jac;
    double* jacobians[] = {0, 0, 0, 0, 0, jac.data()};
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};
    Vec3 residuals;
    f.Evaluate(parameters, residuals.data(), jacobians);

    MatRM3 num_jac = compute_jac(p_b2g, func, 1e-4);

    MATRIX_CLOSE(num_jac, jac, 2e-4);
}

}  // namespace models
}  // namespace mc
