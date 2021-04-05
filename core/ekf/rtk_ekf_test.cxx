
#include "core/ekf/rtk_ekf.h"

#include <gtest/gtest.h>

#include "common/matrix_defs.h"
#include "common/numerical_jacobian.h"
#include "common/random.h"
#include "common/test_helpers.h"
#include "core/sat_manager.h"
#include "utils/wgs84.h"

namespace mc {
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
        MATRIX_CLOSE((x1).sd, (x2).sd, 1e-8);               \
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
        MATRIX_CLOSE((x1).sd, (x2).sd, tol);               \
    }

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

TEST(MeasJacTest, pointPosMeasJacobianTest)
{
    pointPosMeas::ZType z0;
    z0.setRandom();
    const State x0 = State::Random();

    RtkEkf ekf;
    const Input u = Input::Random();

    const auto fun = [&](const State& x) { return ekf.h<pointPosMeas>(z0, x, nullptr, u); };

    const auto fd = compute_jac(x0, fun, 1e-6);

    pointPosMeas::Jac analytical;
    ekf.h<pointPosMeas>(z0, x0, &analytical, u);

    MATRIX_CLOSE(analytical, fd, 1e-8);
}

TEST(MeasJacTest, fixAndHoldMeasJacobianTest)
{
    fixAndHoldMeas::ZType z0;
    z0.setRandom();
    const State x0 = State::Random();

    RtkEkf ekf;

    const auto fun = [&](const State& x) { return ekf.h<fixAndHoldMeas>(z0, x, nullptr); };

    const auto fd = compute_jac(x0, fun, 1e-6);

    fixAndHoldMeas::Jac analytical;
    ekf.h<fixAndHoldMeas>(z0, x0, &analytical);

    MATRIX_CLOSE(analytical, fd, 1e-8);
}

template <typename T>
class ObsMeasJacTest : public ::testing::Test
{
};

typedef ::testing::Types<gpsObsMeas, galObsMeas, gloObsMeas> MeasurementTypes;

TYPED_TEST_SUITE(ObsMeasJacTest, MeasurementTypes);

TYPED_TEST(ObsMeasJacTest, JacobianTest)
{
    typename TypeParam::ZType z0;
    z0.setRandom();
    State x0 = State::Random();

    RtkEkf ekf;
    const auto sat_cache = makeCache();
    const Input u = Input::Random();

    const auto fun = [&](const State& x) { return ekf.h<TypeParam>(z0, x, nullptr, u, sat_cache); };

    const auto fd = compute_jac(x0, fun, 1e-4);

    typename TypeParam::Jac analytical;
    ekf.h<TypeParam>(z0, x0, &analytical, u, sat_cache);

    MATRIX_CLOSE(analytical, fd, 1e-3);
}

TEST(State, BoxplusRules)
{
    const State x = State::Random();
    const State x2 = State::Random();
    const ErrorState zeros = ErrorState::Zero();
    const ErrorState delta1 = ErrorState::Random();

    STATE_EQ(x + zeros, x);
    STATE_EQ(x + (x2 - x), x2);
    MATRIX_CLOSE((x + delta1) - x, delta1, 1e-8);
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
        u += dt * randomNormal<Input>();
        const ErrorState dxdot = RtkEkf::errorStateDynamics(dx, x, u, eta);
        const ErrorState dxdt = RtkEkf::dynamics(x, u, nullptr, nullptr);
        const ErrorState dxhatdt = RtkEkf::dynamics(xhat, u, nullptr, nullptr);

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
        return RtkEkf::errorStateDynamics(dx, x0, u, eta);
    };

    const auto fd = compute_jac(dx0, fun, 1e-8);
    RtkEkf::StateJac A;
    RtkEkf::dynamics(x0, u, &A, nullptr);

    MATRIX_CLOSE(A, fd, 1e-8);
}

TEST(DynamicsTest, InputJacobianTest)
{
    const State x = State::Random();
    const Input u0 = Input::Random();

    const auto fun = [&](const Input& u) -> ErrorState {
        return RtkEkf::dynamics(x, u, nullptr, nullptr);
    };

    const auto fd = compute_jac(u0, fun, 1e-8);
    RtkEkf::InputJac B;
    RtkEkf::dynamics(x, u0, nullptr, &B);

    MATRIX_CLOSE(B, fd, 1e-7);
}

}  // end namespace ekf
}  // end namespace mc
