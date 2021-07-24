
#include "core/ekf/rtk_ekf.h"

#include <gtest/gtest.h>

#include "common/matrix_defs.h"
#include "common/numerical_jacobian.h"
#include "common/random.h"
#include "common/satellite/satellite_cache.h"
#include "common/test_helpers.h"
#include "core/sat_manager.h"
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
    for (int num_sd : {0, 1, 15, 30})
    {
        x0.num_sd = num_sd;

        const auto sat_cache = makeCache();
        const Input u = Input::Random();

        const auto fun = [&](const State& x) { return h<TypeParam>(z0, x, nullptr, u, sat_cache); };

        const Eigen::MatrixXd fd = compute_jac(x0, fun, 1e-4).leftCols(x0.size());

        typename TypeParam::Jac analytical;
        h<TypeParam>(z0, x0, &analytical, u, sat_cache);

        MATRIX_CLOSE(analytical.dense(x0.num_sd), fd, 1e-3);
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
}  // end namespace mc
