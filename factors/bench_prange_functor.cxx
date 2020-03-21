#include <benchmark/benchmark.h>

#include "common/ephemeris/gps.h"
#include "common/matrix_defs.h"
#include "common/satellite/satellite.h"
#include "factors/prange_functor.h"

namespace mc {
namespace factors {

using DQ = math::DQuat<double>;

class PrangeFactor : public benchmark::Fixture
{
 public:
    PrangeFactor() : sat(GnssID::GPS, 1), eph(1)
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

        T_n2b = DQ::identity();
        vel_b = Vec3::Zero();
        clk = Vec2::Zero();
        T_e2r = utils::WGS84::dq_ecef2ned(provo_ecef);
        T_r2n = DQ::identity();
        p_b2g = Vec3::Zero();
        sw = 1.0;

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

BENCHMARK_F(PrangeFactor, computeConstants)(benchmark::State& st)
{
    PseudorangeFactor f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);
    for (auto _ : st)
    {
        f.computeConstants(provo_ecef);
    }
}

BENCHMARK_F(PrangeFactor, Evaluate_NoJac)(benchmark::State& st)
{
    PseudorangeFactor f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);

    Vec3 residuals;
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};
    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), nullptr);
    }
}

BENCHMARK_F(PrangeFactor, Evaluate_PoseJac)(benchmark::State& st)
{
    PseudorangeFactor f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);

    Vec3 residuals;
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};
    MatRM38 jac;
    double* jacobians[] = {jac.data(), 0, 0, 0, 0, 0};
    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(PrangeFactor, Evaluate_VelJac)(benchmark::State& st)
{
    PseudorangeFactor f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);

    Vec3 residuals;
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};
    MatRM3 jac;
    double* jacobians[] = {0, jac.data(), 0, 0, 0, 0};
    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(PrangeFactor, Evaluate_ClkJac)(benchmark::State& st)
{
    PseudorangeFactor f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);

    Vec3 residuals;
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};

    MatRM32 jac;
    double* jacobians[] = {0, 0, jac.data(), 0, 0, 0};
    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(PrangeFactor, Evaluate_RefPoseJac)(benchmark::State& st)
{
    PseudorangeFactor f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);

    Vec3 residuals;
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};

    MatRM38 jac;
    double* jacobians[] = {0, 0, 0, jac.data(), 0, 0};
    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(PrangeFactor, Evaluate_SwitchJac)(benchmark::State& st)
{
    PseudorangeFactor f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);

    Vec3 residuals;
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};

    Vec3 jac;
    double* jacobians[] = {0, 0, 0, 0, jac.data(), 0};
    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(PrangeFactor, Evaluate_LeverArmJac)(benchmark::State& st)
{
    PseudorangeFactor f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);

    Vec3 residuals;
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};

    MatRM3 jac;
    double* jacobians[] = {0, 0, 0, 0, 0, jac.data()};
    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(PrangeFactor, Evaluate_AllJac)(benchmark::State& st)
{
    PseudorangeFactor f(obs, sat, provo_ecef, Mat3::Identity(), T_e2r);

    Vec3 residuals;
    const double* parameters[] = {T_n2b.data(), vel_b.data(), clk.data(),
                                  T_r2n.data(), &sw,          p_b2g.data()};

    MatRM38 pose_jac, ref_pose_jac;
    MatRM3 vel_jac, lever_arm_jac;
    MatRM32 clk_jac;
    Vec3 sw_jac;
    double* jacobians[] = {pose_jac.data(),     vel_jac.data(), clk_jac.data(),
                           ref_pose_jac.data(), sw_jac.data(),  lever_arm_jac.data()};
    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}

}  // namespace factors
}  // namespace mc
