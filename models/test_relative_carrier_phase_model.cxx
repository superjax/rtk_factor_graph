#include <gtest/gtest.h>

#include "common/defs.h"
#include "common/ephemeris/gps.h"
#include "common/matrix_defs.h"
#include "common/numerical_jacobian.h"
#include "common/test_helpers.h"
#include "common/utctime.h"
#include "models/relative_carrier_phase_model.h"
#include "utils/wgs84.h"

namespace mc {
namespace models {

using DQ = math::DQuat<double>;

class TestRelCP : public ::testing::Test
{
 public:
    TestRelCP() : sat(GnssID::GPS, 1), eph(1) {}

    void SetUp() override
    {
        const int week = 86400.00 / UTCTime::SEC_IN_WEEK;
        const int tow_sec = 86400.00 - (week * UTCTime::SEC_IN_WEEK);
        t1 = UTCTime::fromGPS(week, tow_sec * 1000);
        t2 = t1 + 1.0;

        const Vec3 provo_lla{deg2Rad(40.246184), deg2Rad(-111.647769), 1387.997511};
        provo_ecef = utils::WGS84::lla2ecef(provo_lla);

        eph.sat = 1;
        eph.gnssID = GnssID::GPS;
        eph.sqrta = 5153.79589081;
        eph.toe = t1 - 120.0;
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

        T_n2b1 = DQ::Random();
        T_n2b2 = DQ::Random();
        p_b2g = Vec3::Random();
        clk1 = Vec2::Random();
        clk2 = Vec2::Random();
        T_e2r = utils::WGS84::dq_ecef2ned(provo_ecef);
        sw1 = 0.293;
        sw2 = 0.1873;
        T_r2n = DQ(math::Quatd::from_euler(0, 0, 0.234), Vec3::Random());

        const int cphase_bias = rand() % 25000;

        obs1.t = t1;
        obs1.gnss_id = sat.gnssId();
        obs1.sat_num = sat.sat_num();
        obs1.freq = ephemeris::GPSEphemeris::L1;
        obs1.pseudorange = 22080605.29766005;
        obs1.doppler = 0;
        obs1.carrier_phase = obs1.pseudorange * sat.carrierWavelength(obs1.freq) + cphase_bias;

        obs2.t = t2;
        obs2.gnss_id = sat.gnssId();
        obs2.sat_num = sat.sat_num();
        obs2.freq = ephemeris::GPSEphemeris::L1;
        obs2.pseudorange = 22081112.980804175;
        obs2.doppler = 0;
        obs2.carrier_phase = obs2.pseudorange * sat.carrierWavelength(obs1.freq) + cphase_bias;
    }

    void zeroParameters()
    {
        T_n2b1 = DQ(math::Quatd::Identity(), Vec3::Zero());
        T_n2b2 = DQ(math::Quatd::Identity(), Vec3(1, 1, 0));
        p_b2g = Vec3::Zero();
        clk1 = Vec2::Zero();
        clk2 = Vec2::Zero();
        T_e2r = utils::WGS84::dq_ecef2ned(provo_ecef);
        sw1 = 1.0;
        sw2 = 1.0;
        // T_r2n = DQ(math::Quatd::from_euler(0, 0, 0.234), Vec3::Random());
        T_r2n = DQ::identity();
    }

    UTCTime t1;
    UTCTime t2;
    satellite::Satellite<ephemeris::GPSEphemeris> sat;
    ephemeris::GPSEphemeris eph;
    Vec3 provo_ecef;
    meas::GnssObservation obs1;
    meas::GnssObservation obs2;

    DQ T_n2b1;
    DQ T_n2b2;
    Vec2 clk1;
    Vec2 clk2;
    DQ T_e2r;
    DQ T_r2n;
    Vec3 p_b2g;
    double sw1;
    double sw2;
    double Xi = 1.0;
};

TEST_F(TestRelCP, HandCrafted)
{
    zeroParameters();
    const int cphase_bias = rand() % 25000;
    obs1.pseudorange = 22080603.29766005;
    obs1.carrier_phase = obs1.pseudorange * sat.carrierWavelength(obs1.freq) + cphase_bias;
    obs2.pseudorange = 22081196.980804175;
    obs2.carrier_phase = obs2.pseudorange * sat.carrierWavelength(obs1.freq) + cphase_bias;

    RelativeCarrierPhase f(obs1, obs2, sat, provo_ecef, T_e2r, Xi);

    const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                  &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
    double residual;
    EXPECT_TRUE(f.Evaluate(parameters, &residual, nullptr));

    EXPECT_NEAR(residual, 0, 1e-8);
}

TEST_F(TestRelCP, HandCrafted2)
{
    zeroParameters();
    RelativeCarrierPhase f(obs1, obs2, sat, provo_ecef, T_e2r, Xi);

    const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                  &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
    double residual;
    EXPECT_TRUE(f.Evaluate(parameters, &residual, nullptr));

    EXPECT_NEAR(residual, -16.365255860, 1e-8);
}

TEST_F(TestRelCP, Jac_T2nb1)
{
    RelativeCarrierPhase f(obs1, obs2, sat, provo_ecef, T_e2r, Xi);

    const auto fun = [&, this](const Vec8& _T_n2b1) -> Vec1 {
        const double* parameters[] = {_T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                      &sw1,           &sw2,          T_r2n.data(), p_b2g.data()};
        Vec1 res;
        EXPECT_TRUE(f.Evaluate(parameters, res.data(), nullptr));
        return res;
    };

    const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                  &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
    Vec8 jac;
    double* jacobians[] = {jac.data(), 0, 0, 0, 0, 0, 0, 0};

    const auto num_jac = compute_jac(Vec8(T_n2b1.arr_), fun, 1e-4);

    Vec1 res;
    EXPECT_TRUE(f.Evaluate(parameters, res.data(), jacobians));

    MATRIX_CLOSE(num_jac * T_n2b1.dParamDGen(), jac.transpose() * T_n2b1.dParamDGen(),
                 1e-5);
}

TEST_F(TestRelCP, Jac_T2nb2)
{
    RelativeCarrierPhase f(obs1, obs2, sat, provo_ecef, T_e2r, Xi);

    const auto fun = [&, this](const Vec8& _T_n2b2) -> Vec1 {
        const double* parameters[] = {T_n2b1.data(), _T_n2b2.data(), clk1.data(),  clk2.data(),
                                      &sw1,          &sw2,           T_r2n.data(), p_b2g.data()};
        Vec1 res;
        EXPECT_TRUE(f.Evaluate(parameters, res.data(), nullptr));
        return res;
    };

    const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                  &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
    Vec8 jac;
    double* jacobians[] = {0, jac.data(), 0, 0, 0, 0, 0, 0};

    const auto num_jac = compute_jac(Vec8(T_n2b2.arr_), fun, 1e-4);

    Vec1 res;
    EXPECT_TRUE(f.Evaluate(parameters, res.data(), jacobians));

    MATRIX_CLOSE(num_jac * T_n2b2.dParamDGen(), jac.transpose() * T_n2b2.dParamDGen(),
                 1e-5);
}

TEST_F(TestRelCP, JacClk1)
{
    RelativeCarrierPhase f(obs1, obs2, sat, provo_ecef, T_e2r, Xi);

    const auto fun = [&, this](const Vec2& _clk1) -> Vec1 {
        const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), _clk1.data(), clk2.data(),
                                      &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
        Vec1 res;
        EXPECT_TRUE(f.Evaluate(parameters, res.data(), nullptr));
        return res;
    };

    const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                  &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
    Vec2 jac;
    double* jacobians[] = {0, 0, jac.data(), 0, 0, 0, 0, 0};

    const auto num_jac = compute_jac(clk1, fun, 1e-5);

    Vec1 res;
    EXPECT_TRUE(f.Evaluate(parameters, res.data(), jacobians));

    MATRIX_CLOSE(num_jac, jac.transpose(), 1e-4);
}

TEST_F(TestRelCP, JacClk2)
{
    RelativeCarrierPhase f(obs1, obs2, sat, provo_ecef, T_e2r, Xi);

    const auto fun = [&, this](const Vec2& _clk2) -> Vec1 {
        const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  _clk2.data(),
                                      &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
        Vec1 res;
        EXPECT_TRUE(f.Evaluate(parameters, res.data(), nullptr));
        return res;
    };

    const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                  &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
    Vec2 jac;
    double* jacobians[] = {0, 0, 0, jac.data(), 0, 0, 0, 0};

    const auto num_jac = compute_jac(clk1, fun, 1e-5);

    Vec1 res;
    EXPECT_TRUE(f.Evaluate(parameters, res.data(), jacobians));

    MATRIX_CLOSE(num_jac.transpose(), jac, 1e-4);
}

TEST_F(TestRelCP, JacSw1)
{
    RelativeCarrierPhase f(obs1, obs2, sat, provo_ecef, T_e2r, Xi);

    const auto fun = [&, this](const Vec1& _sw1) -> Vec1 {
        const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                      _sw1.data(),   &sw2,          T_r2n.data(), p_b2g.data()};
        Vec1 res;
        EXPECT_TRUE(f.Evaluate(parameters, res.data(), nullptr));
        return res;
    };

    const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                  &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
    Vec1 jac;
    double* jacobians[] = {0, 0, 0, 0, jac.data(), 0, 0, 0};

    Vec1 num_jac = compute_jac(Vec1(sw1), fun, 1e-5);

    Vec1 res;
    EXPECT_TRUE(f.Evaluate(parameters, res.data(), jacobians));

    MATRIX_CLOSE(num_jac, jac, 1e-4);
}

TEST_F(TestRelCP, JacSw2)
{
    RelativeCarrierPhase f(obs1, obs2, sat, provo_ecef, T_e2r, Xi);

    const auto fun = [&, this](const Vec1& _sw2) -> Vec1 {
        const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                      &sw1,          _sw2.data(),   T_r2n.data(), p_b2g.data()};
        Vec1 res;
        EXPECT_TRUE(f.Evaluate(parameters, res.data(), nullptr));
        return res;
    };

    const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                  &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
    Vec1 jac;
    double* jacobians[] = {0, 0, 0, 0, 0, jac.data(), 0, 0};

    Vec1 num_jac = compute_jac(Vec1(sw2), fun, 1e-5);

    Vec1 res;
    EXPECT_TRUE(f.Evaluate(parameters, res.data(), jacobians));

    MATRIX_CLOSE(num_jac, jac, 1e-4);
}

TEST_F(TestRelCP, Jac_Tr2n)
{
    RelativeCarrierPhase f(obs1, obs2, sat, provo_ecef, T_e2r, Xi);

    const auto fun = [&, this](const Vec8& _T_r2n) -> Vec1 {
        const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),   clk2.data(),
                                      &sw1,          &sw2,          _T_r2n.data(), p_b2g.data()};
        Vec1 res;
        EXPECT_TRUE(f.Evaluate(parameters, res.data(), nullptr));
        return res;
    };

    const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                  &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
    Vec8 jac;
    double* jacobians[] = {0, 0, 0, 0, 0, 0, jac.data(), 0};

    const auto num_jac = compute_jac(Vec8(T_r2n.arr_), fun, 1e-4);

    Vec1 res;
    EXPECT_TRUE(f.Evaluate(parameters, res.data(), jacobians));

    MATRIX_CLOSE(num_jac * T_r2n.dParamDGen(), jac.transpose() * T_r2n.dParamDGen(),
                 1e-5);
}

TEST_F(TestRelCP, JacPb2g)
{
    RelativeCarrierPhase f(obs1, obs2, sat, provo_ecef, T_e2r, Xi);

    const auto fun = [&, this](const Vec3& _p_b2g) -> Vec1 {
        const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                      &sw1,          &sw2,          T_r2n.data(), _p_b2g.data()};
        Vec1 res;
        EXPECT_TRUE(f.Evaluate(parameters, res.data(), nullptr));
        return res;
    };

    const double* parameters[] = {T_n2b1.data(), T_n2b2.data(), clk1.data(),  clk2.data(),
                                  &sw1,          &sw2,          T_r2n.data(), p_b2g.data()};
    Vec3 jac;
    double* jacobians[] = {0, 0, 0, 0, 0, 0, 0, jac.data()};

    const auto num_jac = compute_jac(p_b2g, fun, 1e-5);

    Vec1 res;
    EXPECT_TRUE(f.Evaluate(parameters, res.data(), jacobians));

    MATRIX_CLOSE(num_jac, jac.transpose(), 1e-4);
}

}  // namespace models
}  // namespace mc
