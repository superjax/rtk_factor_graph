#include <benchmark/benchmark.h>

#include "common/print.h"
#include "core/ekf/rtk_ekf.h"
#include "core/sat_manager.h"
#include "utils/wgs84.h"

namespace mc {
namespace ekf {

static void dynamics(benchmark::State& state)
{
    RtkEkf ekf;
    const State x = State::Random();
    const Input u = Input::Random();
    RtkEkf::StateJac dxdx;
    RtkEkf::InputJac dxdu;

    for (auto _ : state)
    {
        const ErrorState dx = ekf.dynamics(x, u, &dxdx, &dxdu);
        benchmark::DoNotOptimize(dx);
    }
}
BENCHMARK(dynamics);

template <typename MeasType>
static void h_no_args(benchmark::State& state)
{
    RtkEkf ekf;
    State x = State::Random();
    const typename MeasType::ZType z = MeasType::ZType::Random();
    typename MeasType::Jac jac;

    for (auto _ : state)
    {
        const auto residual = ekf.h<MeasType>(z, x, &jac);
        benchmark::DoNotOptimize(residual);
    }
}
BENCHMARK_TEMPLATE(h_no_args, pointPosMeas);
BENCHMARK_TEMPLATE(h_no_args, fixAndHoldMeas);

const Vec3 provo_lla{deg2Rad(40.246184), deg2Rad(-111.647769), 1387.997511};

satellite::SatelliteCache makeCache()
{
    const int week = 86400.00 / UTCTime::SEC_IN_WEEK;
    const int tow_sec = 86400.00 - (week * UTCTime::SEC_IN_WEEK);
    const UTCTime t = UTCTime::fromGPS(week, tow_sec * 1000);

    const Vec3 provo_ecef = utils::WGS84::lla2ecef(provo_lla);
    const Vec3 rec_vel(1, 2, 3);

    // int toe_week = 93600.0 / UTCTime::SEC_IN_WEEK;
    // int toe_tow_sec = 93600.0 - (toe_week * UTCTime::SEC_IN_WEEK);
    ephemeris::GPSEphemeris eph(1);
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
    satellite::Satellite<ephemeris::GPSEphemeris> sat(GnssID::GPS, 1);
    sat.addEph(eph);

    satellite::SatelliteCache cache;
    cache.update(t, provo_ecef, sat);
    return cache;
}

template <typename MeasType>
static void h_sat_cache(benchmark::State& state)
{
    RtkEkf ekf;
    const auto sat_cache = makeCache();
    State x = State::Random();
    const typename MeasType::ZType z = MeasType::ZType::Random();
    typename MeasType::Jac jac;

    for (auto _ : state)
    {
        const auto residual = ekf.h<MeasType>(z, x, &jac, sat_cache);
        benchmark::DoNotOptimize(residual);
    }
}
BENCHMARK_TEMPLATE(h_sat_cache, gpsObsMeas);
BENCHMARK_TEMPLATE(h_sat_cache, galObsMeas);
BENCHMARK_TEMPLATE(h_sat_cache, gloObsMeas);

static void predict(benchmark::State& state)
{
    RtkEkf ekf;
    ekf.x() = State::Identity();
    ekf.x().t = UTCTime(0);
    const Input u = 1e-5 * Input::Random();
    RtkEkf::ProcessCov Qx;
    Qx.setIdentity();
    RtkEkf::InputCov Qu;
    Qu.setIdentity();
    UTCTime t_new = ekf.x().t + 0.01;

    for (auto _ : state)
    {
        ekf.predict(t_new, u, Qx, Qu);
        t_new += 0.01;
    }
}
BENCHMARK(predict);

template <typename MeasType>
static void update_no_args(benchmark::State& state)
{
    RtkEkf ekf;
    ekf.x() = State::Random();
    const typename MeasType::ZType z = ekf.h<MeasType>(MeasType::ZType::Zero(), ekf.x(), nullptr);
    typename MeasType::Covariance R;
    R.setIdentity();

    for (auto _ : state)
    {
        const Error res = ekf.update<MeasType>(z, R);
        if (!res.ok())
        {
            error("UH Oh");
        }
        benchmark::DoNotOptimize(res);
    }
}
BENCHMARK_TEMPLATE(update_no_args, pointPosMeas);
BENCHMARK_TEMPLATE(update_no_args, fixAndHoldMeas);

template <typename MeasType>
static void update_sat_cache(benchmark::State& state)
{
    RtkEkf ekf;
    ekf.x() = State::Random();
    const auto sat_cache = makeCache();
    const typename MeasType::ZType z =
        ekf.h<MeasType>(MeasType::ZType::Zero(), ekf.x(), nullptr, sat_cache);
    typename MeasType::Covariance R;
    R.setIdentity();

    for (auto _ : state)
    {
        const Error res = ekf.update<MeasType>(z, R, sat_cache);
        if (!res.ok())
        {
            error("UH Oh");
        }
        benchmark::DoNotOptimize(res);
    }
}
BENCHMARK_TEMPLATE(update_sat_cache, gpsObsMeas);
BENCHMARK_TEMPLATE(update_sat_cache, galObsMeas);
BENCHMARK_TEMPLATE(update_sat_cache, gloObsMeas);

}  // namespace ekf
}  // namespace mc
