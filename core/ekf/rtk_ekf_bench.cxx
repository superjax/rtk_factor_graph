#include <benchmark/benchmark.h>

#include "common/print.h"
#include "common/satellite/satellite_cache.h"
#include "core/ekf/rtk_ekf.h"
#include "core/sat_manager.h"
#include "utils/wgs84.h"

namespace mc {
namespace ekf {

static void dynamics(benchmark::State& state)
{
    const State x = State::Random();
    const Input u = Input::Random();
    StateJac dxdx;
    InputJac dxdu;

    for (auto _ : state)
    {
        const ErrorState dx = dynamics(x, u, &dxdx, &dxdu);
        benchmark::DoNotOptimize(dx);
    }
}
BENCHMARK(dynamics);

static void h_pointPos(benchmark::State& state)
{
    State x = State::Random();
    const pointPosMeas::ZType z = pointPosMeas::ZType::Random();
    pointPosMeas::Jac jac;
    const Input u = Input::Zero();

    for (auto _ : state)
    {
        const auto residual = h<pointPosMeas>(z, x, &jac, u);
        benchmark::DoNotOptimize(residual);
    }
}
BENCHMARK(h_pointPos);

static void h_fixAndHold(benchmark::State& state)
{
    State x = State::Random();
    fixAndHoldMeas::ZType z;
    for (int i = 0; i < state.range(0); ++i)
    {
        z.push_back(Vec1::Random());
    }
    fixAndHoldMeas::Jac jac;

    for (auto _ : state)
    {
        const auto residual = h<fixAndHoldMeas>(z, x, &jac);
        benchmark::DoNotOptimize(residual);
    }
}
BENCHMARK(h_fixAndHold)->Arg(1)->Arg(3)->Arg(5)->Arg(15)->Arg(24)->Arg(30);

std::vector<satellite::SatelliteCache> cache = {
    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-3254582.037039849, -17192385.459967356, 20630246.267307647),
            .vel = Vec3(2296.5631514868505, 926.1696416445291, 1175.9684866584887),
            .clk = Vec2(-0.0005890386500659424, -3.637978807091712e-12)},
        -3.9344641590155778,
        satellite::AtmosphericCorrection{2.4423096634745325, 2.101962840640255}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-4501275.567205897, -25796974.798793193, -3957045.012552044),
            .vel = Vec3(275.96904575937987, -506.1439312848634, 3121.9170610988654),
            .clk = Vec2(-3.704511703423144e-05, -1.023181539494544e-12)},
        -6.324946364025884, satellite::AtmosphericCorrection{4.418172068663989, 4.34964258581637}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(10244465.855438046, -11075570.54372843, 21928799.65653879),
            .vel = Vec3(2418.582978464542, 1333.7357215846619, -451.88047962401873),
            .clk = Vec2(2.5172080803837067e-06, 2.160049916710704e-12)},
        -16.139659456219288,
        satellite::AtmosphericCorrection{3.576130160338695, 3.559616621125127}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-9872755.919264179, -18904088.272024937, 15676127.437331123),
            .vel = Vec3(249.08376182667126, -2019.1594628128746, -2227.2059875105415),
            .clk = Vec2(-1.8179402773588956e-05, -5.3432813729159526e-12)},
        2.6125689410885315,
        satellite::AtmosphericCorrection{2.444501605936392, 2.0410339571059404}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-19739526.946187925, -15967457.909055157, -8698756.874315418),
            .vel = Vec3(1090.6833222550615, 170.0490077217778, -2828.6545856989555),
            .clk = Vec2(6.831319376740453e-07, -4.54747350886464e-13)},
        14.774736892541819,
        satellite::AtmosphericCorrection{7.307251502786879, 10.251450287025262}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-14950081.992858456, -4802635.428661016, 21194833.538461965),
            .vel = Vec3(-39.977194273304605, -2748.832288215484, -616.1897582375957),
            .clk = Vec2(0.00010473779011483952, 7.38964445190504e-12)},
        14.379843257043436, satellite::AtmosphericCorrection{3.4665448819014437, 2.91841995306019}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-23082527.072270166, 2384106.853987893, 12914856.229287786),
            .vel = Vec3(1378.3129489723672, -906.7584771287213, 2617.7354654880614),
            .clk = Vec2(-0.0003173077989900265, -6.934897101018576e-12)},
        26.489660456324355,
        satellite::AtmosphericCorrection{6.6253702808270925, 7.169386407161814}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(14746447.054119749, -6680800.338962306, 19776843.08135762),
            .vel = Vec3(-1427.0535701454107, 2285.6755216330453, 1839.7703918457437),
            .clk = Vec2(-5.810149013996124e-05, 0.0)},
        -19.179820224244196,
        satellite::AtmosphericCorrection{4.641032338295951, 6.6163545931696754}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-15066136.991305195, -19477956.33506529, 6640783.754876381),
            .vel = Vec3(-574.4062679458851, -716.6542284765736, -3408.9179984858533),
            .clk = Vec2(6.233993917703629e-05, 0.0)},
        8.086738164156303, satellite::AtmosphericCorrection{3.0322709391994764, 2.54815813335694}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-14500113.60150687, -5795712.638336063, 20184979.83821632),
            .vel = Vec3(-1447.1320244676865, -2351.2250752558407, -1715.4027504139535),
            .clk = Vec2(0.00010476261377334595, -0.0)},
        13.449280080675479, satellite::AtmosphericCorrection{3.325238118769812, 2.792204798635007}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-17965341.235859744, -7573286.126994761, 16476344.568485547),
            .vel = Vec3(2254.533028473108, 172.22303634073594, 2536.017307171217),
            .clk = Vec2(0.0004104766921297906, 2.728484105318784e-12)},
        16.49163777567729, satellite::AtmosphericCorrection{3.413568251526465, 2.829566083910008}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-25448369.22201569, -701993.2198606478, 983309.4604031168),
            .vel = Vec3(130.3698672697887, 238.43505861888747, 3607.896712709652),
            .clk = Vec2(7.080379873514175e-05, 0.0)},
        27.747506171871127,
        satellite::AtmosphericCorrection{9.597582942045392, 25.325431474363025}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(14221926.442285303, -8457812.644489227, 19410136.06151464),
            .vel = Vec3(2665.0951980827417, -98.46994876876103, -1994.1470551484929),
            .clk = Vec2(0.00037399554755425315, 1.818989403545856e-12)},
        -19.379095666917408, satellite::AtmosphericCorrection{4.3855511815084, 5.5709121794777765}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-120560.74300628771, -10830621.074284332, 23112149.53734139),
            .vel = Vec3(3159.7091941594194, 72.49678783482945, 49.94845912635508),
            .clk = Vec2(3.614710775619409e-05, 1.818989403545856e-12)},
        -4.605926377918987,
        satellite::AtmosphericCorrection{2.8095856183749195, 2.495553826243272}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-17183367.42106303, -16668355.591668155, 17412271.48469423),
            .vel = Vec3(1650.1267422383075, 587.7881048251999, 2193.204229625881),
            .clk = Vec2(0.0008231257407733277, -4.2632564145606e-14)},
        11.650117976594256,
        satellite::AtmosphericCorrection{2.6605158958789574, 2.222748839190791}},

    satellite::SatelliteCache{
        Vec3(-1798810.227085327, -4532232.540368021, 4099784.739630998),
        satellite::SatelliteState{
            .t = 1616547814.7112455,
            .pos = Vec3(-10860879.431400301, -26715576.496355664, 6699997.836984555),
            .vel = Vec3(-249.01422135112233, -633.6061132501719, -2922.5679753139925),
            .clk = Vec2(0.0030826305226858905, -2.91464630208793e-11)},
        0.28404912651327635,
        satellite::AtmosphericCorrection{2.8391253145636632, 2.4397771217496045}},
};

static void h_obsMeas(benchmark::State& state)
{
    obsMeas::ZType z;
    std::array<int, 3> gnss_ids = {{GnssID::GPS, GnssID::Galileo, GnssID::Glonass}};
    for (int i = 0; i < state.range(0); ++i)
    {
        z.push_back({Vec2::Random(), cache[i], gnss_ids[i % 3]});
    }

    State x = State::Random();
    typename obsMeas::Jac jac;
    const Input u = Input::Zero();

    for (auto _ : state)
    {
        const auto residual = h<obsMeas>(z, x, &jac, u);
        benchmark::DoNotOptimize(residual);
    }
}
BENCHMARK(h_obsMeas)->Arg(1)->Arg(3)->Arg(5)->Arg(15)->Arg(24)->Arg(30);

static void predict(benchmark::State& state)
{
    Snapshot snap;
    snap.x = State::Identity();
    snap.x.t = UTCTime(0);
    snap.cov.setIdentity();
    Snapshot out;
    const Input u = Input::Random() * 1e-5;
    ProcessCovariance Qx;
    Qx.setIdentity();
    InputCovariance Qu;
    Qu.setZero();
    UTCTime t_new(0.01);

    for (auto _ : state)
    {
        predict(snap, t_new, u, Qx, Qu, make_out(snap));
        t_new += 0.01;
    }
}
BENCHMARK(predict);

static void update_pointPos(benchmark::State& state)
{
    Snapshot snap;
    snap.x = State::Random();
    snap.x.t = UTCTime(0);
    snap.cov.setIdentity();
    const Input u = Input::Zero();

    const typename pointPosMeas::ZType z =
        h<pointPosMeas>(pointPosMeas::ZType::Zero(), snap.x, nullptr, u);
    typename pointPosMeas::Covariance R;
    R.setIdentity();

    for (auto _ : state)
    {
        const auto res = update<pointPosMeas>(make_out(snap), z, R, u);
        if (!res.ok())
        {
            error("UH Oh");
        }
        benchmark::DoNotOptimize(res);
    }
}
BENCHMARK(update_pointPos);

static void update_obsMeas(benchmark::State& state)
{
    obsMeas::ZType z;
    std::array<int, 3> gnss_ids = {{GnssID::GPS, GnssID::Galileo, GnssID::Glonass}};
    for (int i = 0; i < state.range(0); ++i)
    {
        z.push_back({Vec2::Zero(), cache[i], gnss_ids[i % 3]});
    }

    Snapshot snap;
    snap.x = State::Random();
    snap.x.t = UTCTime(0);
    snap.cov.setIdentity();
    typename obsMeas::Jac jac;
    const Input u = Input::Zero();
    typename obsMeas::Covariance R;
    R.setIdentity();

    const typename obsMeas::Residual residual = h<obsMeas>(z, snap.x, nullptr, u);
    for (size_t i = 0; i < z.size(); ++i)
    {
        z[i].z = residual.segment<2>(i * 2);
    }

    for (auto _ : state)
    {
        const auto res = update<obsMeas>(make_out(snap), z, R, u);
        if (!res.ok())
        {
            error("UH Oh");
        }
        benchmark::DoNotOptimize(res);
    }
}
BENCHMARK(update_obsMeas)->Arg(1)->Arg(3)->Arg(5)->Arg(15)->Arg(24)->Arg(30);

static void update_fixAndHold(benchmark::State& state)
{
    fixAndHoldMeas::ZType z;
    for (int i = 0; i < state.range(0); ++i)
    {
        z.push_back(Vec1::Zero());
    }

    Snapshot snap;
    snap.x = State::Random();
    snap.x.t = UTCTime(0);
    snap.cov.setIdentity();
    typename fixAndHoldMeas::Covariance R;
    R.setIdentity();
    fixAndHoldMeas::Jac jac;

    const typename fixAndHoldMeas::Residual residual = h<fixAndHoldMeas>(z, snap.x, nullptr);
    for (size_t i = 0; i < z.size(); ++i)
    {
        z[i] = residual.segment<1>(i);
    }

    for (auto _ : state)
    {
        const auto res = update<fixAndHoldMeas>(make_out(snap), z, R);
        if (!res.ok())
        {
            error("UH Oh");
        }
        benchmark::DoNotOptimize(res);
    }
}
BENCHMARK(update_fixAndHold)->Arg(1)->Arg(3)->Arg(5)->Arg(15)->Arg(24)->Arg(30);

}  // namespace ekf
}  // namespace mc
