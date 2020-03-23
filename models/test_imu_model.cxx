#include <gtest/gtest.h>

#include <random>

#include "common/math/jet.h"
#include "common/numerical_jacobian.h"
#include "common/print.h"
#include "common/random.h"
#include "common/test_helpers.h"
#include "models/imu_model.h"

namespace mc {
namespace models {

#define IMU_STATE_NEAR(x1, x2, tol)            \
    MATRIX_CLOSE((x1).alpha, (x2).alpha, tol); \
    MATRIX_CLOSE((x1).beta, (x2).beta, tol);   \
    MATRIX_CLOSE((x1).beta, (x2).beta, tol);   \
    QUAT_CLOSE((x1).gamma, (x2).gamma, tol)

TEST(ImuModel, integrateRandomImu)
{
    math::Jet<double> x1 = math::Jet<double>::Identity();
    math::Jet<double> x = x1;
    meas::ImuSample sample;
    sample.setZero();

    ImuModel integrator(sample.t, Vec6::Zero());
    integrator.reset(sample.t);

    const Vec3 gravity(0, 0, 9.80665);

    Vec3 accel = -gravity;
    Vec3 omega = Vec3::Zero();

    constexpr double dt = 0.0001;
    for (int i = 0; i < 10000; ++i)
    {
        accel += dt * Vec3::Random();
        omega += dt * Vec3::Random();
        sample.t += dt;
        sample.gyro = omega;
        sample.accel = accel;
        x.x = x.x * math::DQuat<double>::exp(vstack(omega * dt, x.linear_vel * dt));
        x.linear_vel += dt * (accel + x.x.rotation().rotp(gravity));
        x.angular_vel = omega;
        integrator.integrate(sample, Mat6::Identity());
    }

    math::Jet<double> x2;
    integrator.computeEndJet(x1, Out(x2));

    QUATERNION_EQUALS(x.x.rotation(), x2.x.rotation());
    // Numerical error from Euler's method in ImuModel::integrate
    MATRIX_CLOSE(x.x.translation(), x2.x.translation(), 1e-3);
    MATRIX_CLOSE(x.linear_vel, x2.linear_vel, 1e-4);
    MATRIX_EQUALS(x.angular_vel, x2.angular_vel);
}

TEST(ImuModel, NoUpdatesFinished)
{
    ImuModel integrator(UTCTime(0, 0), Vec6::Zero());
    const Error result = integrator.finished();
    EXPECT_TRUE(result.ok());
}

TEST(ImuModel, OneUpdateFinished)
{
    meas::ImuSample sample;
    sample.setRandom();

    ImuModel integrator(sample.t, Vec6::Zero());
    sample.t += 0.01;
    integrator.integrate(sample, 1e-3 * Mat6::Identity());

    const Error result = integrator.finished();
    EXPECT_TRUE(result.ok());
    EXPECT_TRUE(isFinite(integrator.Xi()));
}

TEST(ImuModel, TwoUpdatesFinished)
{
    meas::ImuSample sample;
    sample.setRandom();

    ImuModel integrator(sample.t, Vec6::Zero());
    sample.t += 0.01;
    integrator.integrate(sample, 1e-3 * Mat6::Identity());
    sample.t += 0.01;
    integrator.integrate(sample, 1e-3 * Mat6::Identity());

    const Error result = integrator.finished();
    EXPECT_TRUE(result.ok());
    EXPECT_TRUE(isFinite(integrator.Xi()));
}

TEST(ImuModel, ErrorStateDynamics)
{
    const UTCTime t0(0, 0);
    const double Tmax = 0.01;
    static const double dt = 0.001;

    const Vec6 bias = Vec6::Zero();
    ImuModel y(t0, bias);
    ImuModel yhat(t0, bias);
    yhat.state() = y.state() + Vec9::Constant(0.01);
    ImuErrorState dy = y.state() - yhat.state();

    ImuState y_check = yhat.state() + dy;
    IMU_STATE_NEAR(y.state(), y_check, 1e-8);

    meas::ImuSample u;
    const Vec6 eta = Vec6::Zero();
    u.t = t0;
    u.setZero();

    std::default_random_engine gen;
    std::normal_distribution<double> normal;

    const Mat6 cov = Mat6::Identity() * 1e-3;
    ImuErrorState dydot;
    for (int i = 0; i < Tmax / dt; i++)
    {
        u.accel += dt * randomNormal<Vec3>();
        u.gyro += dt * randomNormal<Vec3>();
        u.t += dt;
        y.errorStateDynamics(y.state(), dy, u, eta, Out(dydot));
        dy += dydot * dt;

        y.integrate(u, cov);
        yhat.integrate(u, cov);
        y_check = yhat.state() + dy;
        const double t = u.t.toSec();
        IMU_STATE_NEAR(y.state(), y_check, t > 0.3 ? 5e-6 * t * t : 2e-7);
    }
}

TEST(ImuFactor, DynamicsJacobians)
{
    Mat6 cov = Mat6::Identity() * 1e-3;

    Vec6 b0;
    ImuState y0;
    meas::ImuSample u0;
    Vec6 eta0;
    ImuErrorState ydot;
    ImuErrorState dy0;

    Mat9 A;
    Mat96 B;

    for (int i = 0; i < 100; i++)
    {
        b0.setRandom();
        y0.setRandom();
        u0.setRandom();

        eta0.setZero();
        dy0.setZero();

        ImuModel f(UTCTime(0, 0), b0);
        f.dynamics(y0, u0, Out(ydot), Out(A), Out(B));

        auto yfun = [&y0, &cov, &b0, &u0, &eta0](const ImuErrorState& dy) {
            ImuModel functor(UTCTime(0, 0), b0);
            ImuErrorState dydot;
            functor.errorStateDynamics(y0, dy, u0, eta0, Out(dydot));
            return dydot;
        };
        auto etafun = [&y0, &cov, &b0, &dy0, &u0](const Vec6& eta) {
            ImuModel functor(0, b0);
            ImuErrorState dydot;
            functor.errorStateDynamics(y0, dy0, u0, eta, Out(dydot));
            return dydot;
        };
        (void)etafun;

        const Mat9 AFD = compute_jac(dy0, yfun);
        const Mat96 BFD = compute_jac(eta0, etafun);

        MATRIX_CLOSE(AFD, A, 1e-7);
        MATRIX_CLOSE(BFD, B, 1e-7);
    }
}

TEST(ImuFactor, BiasJacobians)
{
    UTCTime t(0, 0);
    const UTCTime t0(0, 0);
    std::vector<meas::ImuSample> imu_samples;
    while (t.toSec() < 0.1)
    {
        meas::ImuSample sample;
        sample.t = t;
        sample.accel.setRandom();
        sample.gyro.setRandom();
        imu_samples.push_back(std::move(sample));
        t += 0.001;
    }

    Mat6 cov = Mat6::Identity() * 1e-3;
    Vec6 b0;
    Mat96 JFD;

    b0.setZero();
    ImuModel f(0, b0);
    ImuState y0 = f.state();
    for (const auto sample : imu_samples)
    {
        f.integrate(sample, cov);
    }
    const Mat96 J = f.biasJacobian();

    auto fun = [&cov, &imu_samples, &t, &y0](const Vec6& bias0) {
        ImuModel functor(0, bias0);
        for (const auto sample : imu_samples)
        {
            functor.integrate(sample, cov);
        }
        return functor.state();
    };

    JFD = compute_jac(b0, fun);
    MATRIX_CLOSE(J, JFD, 1e-4);
}

class ImuFactorJacobian : public ::testing::Test
{
 public:
    ImuFactorJacobian() : integrator(UTCTime(0, 0), Vec6::Zero()) {}

    void SetUp() override
    {
        const Vec3 gravity(0, 0, 9.80665);
        meas::ImuSample sample;
        sample.setZero();
        Vec3 accel = -gravity;
        Vec3 omega = Vec3::Zero();

        constexpr double dt = 0.0001;
        for (int i = 0; i < 10000; ++i)
        {
            accel += dt * Vec3::Random();
            omega += dt * Vec3::Random();
            sample.t += dt;
            sample.gyro = omega;
            sample.accel = accel;
            integrator.integrate(sample, Mat6::Identity());
        }

        start_pose = math::DQuat<double>::Random();
        end_pose = math::DQuat<double>::Random();
        start_vel = Vec3::Random();
        end_vel = Vec3::Random();
        bias = Vec6::Random();
    }

    ImuModel integrator;
    math::DQuat<double> start_pose;
    math::DQuat<double> end_pose;
    Vec3 start_vel;
    Vec3 end_vel;
    Vec6 bias;
};

TEST_F(ImuFactorJacobian, dRes_dStartPose)
{
    auto fun = [this](const Vec8& _start_pose, double* _jac) {
        const double* parameters[] = {_start_pose.data(), end_pose.data(), start_vel.data(),
                                      end_vel.data(), bias.data()};
        double* jacobians[] = {
            _jac, nullptr, nullptr, nullptr, nullptr,
        };
        Vec9 residuals;
        EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), jacobians));
        return residuals;
    };

    /// TODO:  Think about making this not just at identity
    start_pose = math::DQuat<double>::identity();

    MatRM98 J;
    fun(start_pose.arr_, J.data());

    Mat98 JFD = compute_jac(Vec8(start_pose.arr_), fun, nullptr);

    MATRIX_CLOSE(J.rightCols(7), JFD.rightCols(7), 1e-6);
}

TEST_F(ImuFactorJacobian, dRes_dEndPose)
{
    auto fun = [this](const Vec8& _end_pose, double* _jac) {
        const double* parameters[] = {start_pose.data(), _end_pose.data(), start_vel.data(),
                                      end_vel.data(), bias.data()};
        double* jacobians[] = {
            nullptr, _jac, nullptr, nullptr, nullptr,
        };
        Vec9 residuals;
        EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), jacobians));
        return residuals;
    };

    /// TODO:  Think about making this not just at identity
    end_pose = math::DQuat<double>::identity();

    MatRM98 J;
    fun(end_pose.arr_, J.data());

    Mat98 JFD = compute_jac(Vec8(end_pose.arr_), fun, nullptr);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

TEST_F(ImuFactorJacobian, dRes_dStartVel)
{
    auto fun = [this](const Vec3& _start_vel, double* _jac) {
        const double* parameters[] = {start_pose.data(), end_pose.data(), _start_vel.data(),
                                      end_vel.data(), bias.data()};
        double* jacobians[] = {
            nullptr, nullptr, _jac, nullptr, nullptr,
        };
        Vec9 residuals;
        EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), jacobians));
        return residuals;
    };

    MatRM93 J;
    fun(start_vel, J.data());

    Mat93 JFD = compute_jac(start_vel, fun, nullptr);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

TEST_F(ImuFactorJacobian, dRes_dEndVel)
{
    auto fun = [this](const Vec3& _end_vel, double* _jac) {
        const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                      _end_vel.data(), bias.data()};
        double* jacobians[] = {
            nullptr, nullptr, nullptr, _jac, nullptr,
        };
        Vec9 residuals;
        EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), jacobians));
        return residuals;
    };

    MatRM93 J;
    fun(end_vel, J.data());

    Mat93 JFD = compute_jac(end_vel, fun, nullptr);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

TEST_F(ImuFactorJacobian, dAdjustedState_dAdjustment)
{
    auto fun = [this](const Vec9& bias_adjustment, double* _jac) {
        ImuState adjusted_state = integrator.state() + bias_adjustment;
        return adjusted_state;
    };

    ImuErrorState bias_adjustment = integrator.biasJacobian() * bias;

    Mat3 dexp;
    math::Quat<double>::exp(bias_adjustment.gamma, &dexp);

    Mat9 J;
    J.setZero();
    J.block<3, 3>(0, 0).setIdentity();
    J.block<3, 3>(3, 3).setIdentity();
    J.block<3, 3>(6, 6) = dexp.transpose();

    Mat9 JFD = compute_jac(bias_adjustment, fun, nullptr);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

TEST_F(ImuFactorJacobian, dAdjustedState_dBias)
{
    auto fun = [this](const Vec6& _bias, double* _jac) {
        const Vec9 bias_adjustment = integrator.biasJacobian() * _bias;
        ImuState adjusted_state = integrator.state() + bias_adjustment;
        return adjusted_state;
    };

    ImuErrorState bias_adjustment = integrator.biasJacobian() * bias;

    Mat3 dexp;
    math::Quat<double>::exp(bias_adjustment.gamma, &dexp);

    Mat9 d_adjusted_by_state;
    d_adjusted_by_state.setZero();
    d_adjusted_by_state.block<3, 3>(0, 0).setIdentity();
    d_adjusted_by_state.block<3, 3>(3, 3).setIdentity();
    d_adjusted_by_state.block<3, 3>(6, 6) = dexp.transpose();

    Mat96 J = d_adjusted_by_state * integrator.biasJacobian();

    Mat96 JFD = compute_jac(bias, fun, nullptr);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

template <typename Fun, typename... Args>
Eigen::MatrixXd compute_jac_imu(const ImuState& x, const Fun& fun, const Args&... args)
{
    const double eps = 1e-8;
    Eigen::MatrixXd out;
    for (int i = 0; i < ImuState::DOF; ++i)
    {
        const ImuState xp = x + (ImuErrorState::Unit(i) * eps);
        const ImuState xm = x + (ImuErrorState::Unit(i) * -eps);
        const Vec9 ym = fun(xm, args...);
        const Vec9 yp = fun(xp, args...);

        if (out.rows() == 0 || out.cols() == 0)
        {
            out.resize(decltype(yp - yp)::RowsAtCompileTime, ImuState::DOF);
        }

        out.col(i) = (yp - ym) / (2.0 * eps);
    }
    return out;
}

TEST_F(ImuFactorJacobian, dRes_dAdjusted)
{
    info("");
    auto fun = [this](const ImuState& adjusted_state) {
        Vec9 residual;
        const Vec3 GRAVITY = Vec3::Unit(2) * 9.80665;
        const double _dt = 0.1;
        residual.segment<3>(0) =
            start_pose.rotation().rotp(end_pose.translation() - start_pose.translation() -
                                       0.5 * GRAVITY * _dt * _dt) -
            start_vel * _dt - adjusted_state.alpha;
        residual.segment<3>(3) = adjusted_state.gamma.rota(end_vel) - start_vel -
                                 start_pose.rotation().rotp(GRAVITY) * _dt - adjusted_state.beta;
        residual.segment<3>(6) = (adjusted_state.gamma.inverse() *
                                  (start_pose.rotation().inverse() * end_pose.rotation()))
                                     .log();
        return residual;
    };

    info("");
    ImuState adjusted_state;
    adjusted_state.setRandom();

    info("");
    Mat3 log_jac;
    math::Quat<double> rot_error =
        (adjusted_state.gamma.inverse() * (start_pose.rotation().inverse() * end_pose.rotation()));
    rot_error.log(&log_jac);

    info("");
    Mat9 J;
    J.block<3, 3>(0, 0) = -Mat3::Identity();
    J.block<3, 3>(3, 0).setZero();
    J.block<3, 3>(6, 0).setZero();

    J.block<3, 3>(0, 3).setZero();
    J.block<3, 3>(3, 3) = -Mat3::Identity();
    J.block<3, 3>(6, 3).setZero();

    J.block<3, 3>(0, 6).setZero();
    J.block<3, 3>(3, 6) = -adjusted_state.gamma.R().transpose() * skew(end_vel);
    J.block<3, 3>(6, 6) = -(rot_error.Ad() * log_jac).transpose();

    info("");
    const Mat9 JFD = compute_jac_imu(adjusted_state, fun);
    info("");

    MATRIX_CLOSE(J, JFD, 1e-6);
}

TEST_F(ImuFactorJacobian, dRes_dBias)
{
    auto fun = [this](const Vec6& _bias, double* _jac) {
        const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                      end_vel.data(), _bias.data()};
        double* jacobians[] = {nullptr, nullptr, nullptr, nullptr, _jac};
        Vec9 residuals;
        EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), jacobians));
        return residuals;
    };

    MatRM96 J;
    fun(bias, J.data());

    Mat96 JFD = compute_jac(bias, fun, nullptr);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

}  // namespace models
}  // namespace mc
