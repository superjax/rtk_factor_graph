
#include "core/ekf/rtk_ekf.h"

#include <gtest/gtest.h>

#include "common/matrix_defs.h"
#include "common/numerical_jacobian.h"
#include "common/random.h"
#include "common/satellite/satellite_cache.h"
#include "common/test_helpers.h"
#include "core/sat_manager.h"
#include "sim/sensors/gnss.h"
#include "utils/wgs84.h"

namespace mc {

using ErrVec = Eigen::Matrix<double, ekf::State::DOF, 1>;

template <>
struct PerturbHelper<JacobianSide::LEFT, ekf::State, ErrVec>
{
    static ekf::State perturb(const ekf::State& x, const ErrVec& _dx)
    {
        ekf::ErrorState dx = ekf::ErrorState::fromDense(_dx);
        ekf::State out = x + dx;
        return out;
    }
};

template <>
struct PerturbHelper<JacobianSide::LEFT, ekf::Input, Vec6>
{
    static ekf::Input perturb(const ekf::Input& x, const Vec6& _dx)
    {
        ekf::Input dx = ekf::Input::fromDense(_dx);
        ekf::Input out = x + dx;
        return out;
    }
};

template <>
struct PerturbHelper<JacobianSide::LEFT, ekf::ErrorState, ErrVec>
{
    static ekf::ErrorState perturb(const ekf::ErrorState& x, const ErrVec& _dx)
    {
        ekf::ErrorState dx = ekf::ErrorState::fromDense(_dx);
        return x + dx;
    }
};

template <>
struct DiffHelper<JacobianSide::LEFT, ekf::ErrorState, ekf::ErrorState>
{
    static ErrVec difference(const ekf::ErrorState& x2, const ekf::ErrorState& x1)
    {
        return (x2 - x1).dense(ekf::State::MAX_SD);
    }
};

namespace ekf {
#define STATE_EQ(x1, x2)                                    \
    {                                                       \
        DQUAT_EQ((x1).pose, (x2).pose);                     \
        MATRIX_CLOSE((x1).vel, (x2).vel, 1e-8);             \
        MATRIX_CLOSE((x1).acc_bias, (x2).acc_bias, 1e-8);   \
        MATRIX_CLOSE((x1).gyro_bias, (x2).gyro_bias, 1e-8); \
        MATRIX_CLOSE((x1).gps_clk, (x2).gps_clk, 1e-8);     \
        MATRIX_CLOSE((x1).gal_clk, (x2).gal_clk, 1e-8);     \
        MATRIX_CLOSE((x1).glo_clk, (x2).glo_clk, 1e-8);     \
        MATRIX_CLOSE((x1).p_b2g, (x2).p_b2g, 1e-8);         \
        DQUAT_EQ((x1).T_I2e, (x2).T_I2e);                   \
        EXPECT_EQ(x1.num_sd, x2.num_sd);                    \
        for (int i = 0; i < x1.num_sd; ++i)                 \
        {                                                   \
            MATRIX_CLOSE((x1).sd[i], (x2).sd[i], 1e-8);     \
        }                                                   \
    }

#define STATE_CLOSE(x1, x2, tol)                           \
    {                                                      \
        DQUAT_CLOSE((x1).pose, (x2).pose, tol);            \
        MATRIX_CLOSE((x1).vel, (x2).vel, tol);             \
        MATRIX_CLOSE((x1).acc_bias, (x2).acc_bias, tol);   \
        MATRIX_CLOSE((x1).gyro_bias, (x2).gyro_bias, tol); \
        MATRIX_CLOSE((x1).gps_clk, (x2).gps_clk, tol);     \
        MATRIX_CLOSE((x1).gal_clk, (x2).gal_clk, tol);     \
        MATRIX_CLOSE((x1).glo_clk, (x2).glo_clk, tol);     \
        MATRIX_CLOSE((x1).p_b2g, (x2).p_b2g, tol);         \
        DQUAT_CLOSE((x1).T_I2e, (x2).T_I2e, tol);          \
        EXPECT_EQ(x1.num_sd, x2.num_sd);                   \
        for (size_t i = 0; i < x1.num_sd; ++i)             \
        {                                                  \
            MATRIX_CLOSE((x1).sd[i], (x2).sd[i], tol);     \
        }                                                  \
    }

const Vec3 provo_lla{deg2Rad(40.246184), deg2Rad(-111.647769), 1387.997511};

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

TEST(MeasJacTest, pointPosMeasJacobianTest)
{
    pointPosMeas::ZType z0;
    z0.setRandom();
    State x0 = State::Random();
    const Input u = Input::Random();
    const auto fun = [&](const State& x) { return h<pointPosMeas>(z0, x, nullptr, u); };

    for (int num_sd : {0, 1, 15, 30})
    {
        x0.num_sd = num_sd;

        const Eigen::MatrixXd fd = compute_jac(x0, fun, 1e-6).leftCols(x0.size());

        pointPosMeas::Jac analytical;
        h<pointPosMeas>(z0, x0, &analytical, u);

        MATRIX_CLOSE(analytical.dense(x0.num_sd), fd, 1e-8);
    }
}

TEST(MeasJacTest, fixAndHoldMeasJacobianTest)
{
    State x0 = State::Random();

    for (int num_sd : {0, 1, 15, 30})
    {
        fixAndHoldMeas::ZType z0(num_sd);
        for (auto& z : z0)
        {
            z << ((double)rand() / (double)RAND_MAX);
        }
        x0.num_sd = num_sd;

        const auto fun = [&](const State& x) { return h<fixAndHoldMeas>(z0, x, nullptr); };

        const Eigen::MatrixXd fd = compute_jac(x0, fun, 1e-6).topLeftCorner(num_sd, x0.size());

        fixAndHoldMeas::Jac analytical;
        h<fixAndHoldMeas>(z0, x0, &analytical);

        MATRIX_CLOSE(analytical.dense(x0.num_sd, num_sd), fd, 1e-8);
    }
}

TEST(MeasJacTest, obsMeasJacobianTest)
{
    obsMeas::ZType z0;
    std::array<int, 3> gnss_ids = {{GnssID::GPS, GnssID::Galileo, GnssID::Glonass}};
    for (int i = 0; i < 15; ++i)
    {
        z0.push_back({Vec2::Random(), cache[i], gnss_ids[i % 3]});
    }

    State x0 = State::Random();
    for (int num_sd : {0, 1, 15, 30})
    {
        x0.num_sd = num_sd;

        const Input u = Input::Random();

        const auto fun = [&](const State& x) { return h<obsMeas>(z0, x, nullptr, u); };

        const Eigen::MatrixXd fd =
            compute_jac(x0, fun, 1e-4).topLeftCorner(2 * z0.size(), x0.size());

        typename obsMeas::Jac analytical;
        h<obsMeas>(z0, x0, &analytical, u);

        MATRIX_CLOSE(analytical.dense(z0.size(), x0.num_sd), fd, 1e-3);
    }
}

TEST(DynamicsTest, ErrorStateDynamics)
{
    srand(9);
    State xhat = State::Random();
    State x = xhat + ErrorState::Constant(1e-5);
    ErrorState dx = x - xhat;
    State x_check = xhat + dx;

    const Input eta = Input::Zero();

    static constexpr int MAX_ITERS = 250;
    static constexpr double dt = 0.001;
    Input u = Input::Zero();
    for (double t = 0; t < MAX_ITERS * dt; t += dt)
    {
        u += randomNormal<Input>() * dt;
        const ErrorState dxdot = errorStateDynamics(dx, x, u, eta);
        const ErrorState dxdt = dynamics(x, u, nullptr, nullptr);
        const ErrorState dxhatdt = dynamics(xhat, u, nullptr, nullptr);

        // integrate the error-state
        dx += dxdot * dt;
        // integrate the true state
        x += dxdt * dt;
        // integrate the estimated state
        xhat += dxhatdt * dt;

        // Check that they match
        x_check = xhat + dx;
        const double tol = t > 0.3 ? 5e-4 * t * t : 3e-5;
        STATE_CLOSE(x, x_check, tol);
    }
}

TEST(DynamicsTest, StateJacobianTest)
{
    const State x0 = State::Random();
    const ErrorState dx0 = ErrorState::Random() * 1e-8;
    const Input u = Input::Random();
    const Input eta = Input::Random() * 1e-8;

    const auto fun = [&](const ErrorState& dx) -> ErrorState {
        return errorStateDynamics(dx, x0, u, eta);
    };

    const Eigen::MatrixXd fd = compute_jac(dx0, fun, 1e-8).topLeftCorner(x0.size(), x0.size());
    StateJac A;
    dynamics(x0, u, &A, nullptr);

    MATRIX_CLOSE(A.dense(x0.num_sd), fd, 1e-8);
}

TEST(DynamicsTest, InputJacobianTest)
{
    const State x = State::Random();
    const Input u0 = Input::Random();

    const auto fun = [&](const Input& u) -> ErrorState { return dynamics(x, u, nullptr, nullptr); };

    const Eigen::MatrixXd fd = compute_jac(u0, fun, 1e-8).topRows(x.size());
    InputJac B;
    dynamics(x, u0, nullptr, &B);

    MATRIX_CLOSE(B.dense(x.num_sd), fd, 1e-7);
}

}  // namespace ekf
}  // namespace mc
