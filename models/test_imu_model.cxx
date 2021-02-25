#include <gtest/gtest.h>

#include <random>

#include "common/logging/log_writer.h"
#include "common/math/jet.h"
#include "common/numerical_jacobian.h"
#include "common/print.h"
#include "common/quantized_time.h"
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

#define IMU_STATE_EQ(x1, x2)                    \
    MATRIX_CLOSE((x1).alpha, (x2).alpha, 1e-8); \
    MATRIX_CLOSE((x1).beta, (x2).beta, 1e-8);   \
    MATRIX_CLOSE((x1).beta, (x2).beta, 1e-8);   \
    QUAT_CLOSE((x1).gamma, (x2).gamma, 1e-8)

TEST(ImuModel, integrateRandomImu)
{
    math::Jet<double> x = math::Jet<double>::Identity();
    math::DQuat<double> x1 = x.x;
    Vec3 v1 = x.dx.linear();
    meas::ImuSample sample;
    sample.setZero();
    const Vec6 R = Vec6::Ones();

    const Vec3 gravity(0, 0, 9.80665);

    logging::Logger logger("/tmp/ImuModel/integrateRandomImu.log");
    logger.initStream<math::Jet<double>, math::DQuat<double>, math::DQuat<double>::TangentVector>(
        1, {"x", "xhat", "vhat"});

    Vec3 accel = -gravity;
    Vec3 omega = Vec3::Zero();
    sample.accel = accel;
    sample.gyro = omega;
    ImuModel integrator(sample, Vec6::Zero(), R);
    constexpr double dt = 0.0001;
    Vec3 prev_accel = accel;
    Vec3 prev_omega = omega;
    for (int i = 0; i < 10000; ++i)
    {
        accel += dt * Vec3::Random();
        omega += dt * Vec3::Random();
        sample.t += dt;
        sample.gyro = omega;
        sample.accel = accel;

        Vec3 avg_omega = (prev_omega + omega) / 2.0;
        Vec3 avg_accel = (prev_accel + accel) / 2.0;
        x.x = x.x * math::DQuat<double>::exp(vstack(avg_omega * dt, x.dx.linear() * dt));
        x.dx.linear() += dt * (avg_accel + x.x.rotation().rotp(gravity));
        x.dx.angular() = avg_omega;

        prev_omega = omega;
        prev_accel = accel;

        integrator.integrate(sample);
        math::DQuat<double> x2;
        Vec3 v2;
        integrator.computeEndState(x1, v1, Out(x2), Out(v2));

        logger.log(1, sample.t, x, x2, v2);
    }

    math::DQuat<double> x2;
    Vec3 v2;
    integrator.computeEndState(x1, v1, Out(x2), Out(v2));

    QUAT_EQ(x.x.rotation(), x2.rotation());
    // Numerical error from Euler's method in ImuModel::integrate
    MATRIX_CLOSE(x.x.translation(), x2.translation(), 1e-3);
    MATRIX_CLOSE(x.dx.linear(), v2, 1e-4);
}

TEST(ImuModel, NoUpdatesFinished)
{
    meas::ImuSample sample;
    sample.setZero();
    const Vec6 R = Vec6::Ones();
    ImuModel integrator(sample, Vec6::Zero(), R);
    EXPECT_CHECK_FAIL(integrator.finished(), "");
}

TEST(ImuModel, OneUpdateFinished)
{
    meas::ImuSample sample;
    sample.setRandom();

    const Vec6 R = Vec6::Ones();

    ImuModel integrator(sample, Vec6::Zero(), R);
    sample.t += 0.01;
    integrator.integrate(sample);

    const Error result = integrator.finished();
    EXPECT_TRUE(result.ok());
    EXPECT_TRUE(isFinite(integrator.Xi()));
}

TEST(ImuModel, TwoUpdatesFinished)
{
    meas::ImuSample sample;
    sample.setRandom();
    const Vec6 R = 1e-3 * Vec6::Ones();

    ImuModel integrator(sample, Vec6::Zero(), R);
    sample.t += 0.01;
    integrator.integrate(sample);
    sample.t += 0.01;
    integrator.integrate(sample);

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
    const Vec6 R = 1e-3 * Vec6::Ones();

    meas::ImuSample u;
    u.t = t0;
    u.setZero();

    ImuModel y(u, bias, R);
    ImuModel yhat(u, bias, R);
    yhat.state() = y.state() + Vec9::Constant(0.01);
    ImuErrorState dy = y.state() - yhat.state();

    ImuState y_check = yhat.state() + dy;
    IMU_STATE_NEAR(y.state(), y_check, 1e-8);

    const Vec6 eta = Vec6::Zero();

    std::default_random_engine gen;
    std::normal_distribution<double> normal;

    ImuErrorState dydot;
    for (int i = 0; i < Tmax / dt; i++)
    {
        u.accel += dt * randomNormal<Vec3>();
        u.gyro += dt * randomNormal<Vec3>();
        u.t += dt;
        y.errorStateDynamics(y.state(), dy, u, eta, Out(dydot));
        dy += dydot * dt;

        y.integrate(u);
        yhat.integrate(u);
        y_check = yhat.state() + dy;
        const double t = u.t.toSec();
        IMU_STATE_NEAR(y.state(), y_check, t > 0.3 ? 5e-6 * t * t : 2e-7);
    }
}

TEST(ImuFactor, DynamicsJacobians)
{
    Vec6 b0;
    ImuState y0;
    meas::ImuSample u0;
    Vec6 eta0;
    ImuErrorState ydot;
    ImuErrorState dy0;

    const Vec6 R = Vec6::Ones();

    Mat9 A;
    Mat96 B;

    for (int i = 0; i < 100; i++)
    {
        b0.setRandom();
        y0.setRandom();
        u0.setRandom();

        eta0.setZero();
        dy0.setZero();

        ImuModel f(u0, b0, R);
        f.dynamics(y0, u0, Out(ydot), Out(A), Out(B));

        auto yfun = [&](const ImuErrorState& dy) {
            ImuModel functor(u0, b0, R);
            ImuErrorState dydot;
            functor.errorStateDynamics(y0, dy, u0, eta0, Out(dydot));
            return dydot;
        };
        auto etafun = [&](const Vec6& eta) {
            ImuModel functor(u0, b0, R);
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

    Vec6 cov = Vec6::Ones() * 1e-3;
    Vec6 b0;
    Mat96 JFD;

    b0.setZero();
    ImuModel f(imu_samples.front(), b0, cov);
    for (size_t i = 1; i < imu_samples.size(); ++i)
    {
        const auto sample = imu_samples[i];
        f.integrate(sample);
    }
    const Mat96 J = f.dStatedBias();

    auto fun = [&](const Vec6& bias0) {
        ImuModel functor(imu_samples.front(), bias0, cov);
        for (size_t i = 1; i < imu_samples.size(); ++i)
        {
            const auto sample = imu_samples[i];
            functor.integrate(sample);
        }
        return functor.state();
    };

    JFD = compute_jac(b0, fun);
    MATRIX_CLOSE(J, JFD, 1e-4);
}

class SplitTest
{
    SplitTest() {}
};

TEST(ImuFactor, Split)
{
    UTCTime t(0, 0);
    const UTCTime t0(0, 0);
    const UTCTime tsplit = t0 + 0.01;
    std::vector<meas::ImuSample> imu_samples;
    while (t.toSec() < 0.02)
    {
        meas::ImuSample sample;
        sample.t = t;
        sample.accel.setRandom();
        sample.gyro.setRandom();
        imu_samples.push_back(std::move(sample));
        t += 0.001;
    }

    const Vec6 cov = Vec6::Ones() * 1e-3;
    const Vec6 b0 = Vec6::Zero();

    const math::DQuat<double> x0 = math::DQuat<double>::Random();
    const Vec3 v0 = Vec3::Random();

    ImuModel f(imu_samples.front(), b0, cov);
    ImuState y_split;
    math::DQuat<double> x_split, xm;
    Vec3 v_split, vm;
    for (size_t i = 1; i < imu_samples.size(); ++i)
    {
        const auto sample = imu_samples[i];
        f.integrate(sample);
        if (f.tf().quantized(0.0001) == tsplit)
        {
            y_split = f.state();
            f.computeEndState(x0, v0, Out(xm), Out(vm));
        }
    }

    math::DQuat<double> xf;
    Vec3 vf;
    f.computeEndState(x0, v0, Out(xf), Out(vf));
    EXPECT_EQ(f.tf(), t0 + 0.019);

    ImuModel f2 = f.split(tsplit);
    EXPECT_EQ(f.t0(), t0);
    EXPECT_EQ(f.tf(), f2.t0());
    EXPECT_EQ(f.tf(), tsplit);
    EXPECT_EQ(f2.tf(), t0 + 0.019);

    math::DQuat<double> xm_hat, xf_hat;
    Vec3 vm_hat, vf_hat;
    f.computeEndState(x0, v0, Out(xm_hat), Out(vm_hat));
    f2.computeEndState(xm_hat, vm_hat, Out(xf_hat), Out(vf_hat));

    DQUAT_EQ(xf_hat, xf);
    DQUAT_EQ(xm, xm_hat);
    MAT_EQ(vf_hat, vf);
    MAT_EQ(vm_hat, vm);
    IMU_STATE_EQ(y_split, f.state());
}

TEST(ImuFactor, SplitMiddle)
{
    UTCTime t(0, 0);
    const UTCTime t0(0, 0);
    const UTCTime tsplit = t0 + 0.15;
    std::vector<meas::ImuSample> imu_samples;
    while (t.toSec() < 0.2)
    {
        meas::ImuSample sample;
        sample.t = t;
        sample.accel = -GRAVITY;
        sample.gyro.setZero();
        imu_samples.push_back(std::move(sample));
        t += 0.02;
    }

    const Vec6 cov = Vec6::Ones() * 1e-3;
    const Vec6 b0 = Vec6::Zero();

    const math::DQuat<double> x0 = math::DQuat<double>::identity();
    const Vec3 v0 = Vec3::Zero();

    ImuModel f(imu_samples.front(), b0, cov);
    math::DQuat<double> x_split, xm;
    Vec3 v_split, vm;
    for (size_t i = 1; i < imu_samples.size(); ++i)
    {
        const auto sample = imu_samples[i];
        f.integrate(sample);
    }

    math::DQuat<double> xf1;
    Vec3 vf1;
    f.computeEndState(x0, v0, Out(xf1), Out(vf1));

    ImuModel f2 = f.split(tsplit);

    math::DQuat<double> xm1_hat, xf1_hat;
    Vec3 vm1_hat, vf1_hat;
    f.computeEndState(x0, v0, Out(xm1_hat), Out(vm1_hat));
    f2.computeEndState(xm1_hat, vm1_hat, Out(xf1_hat), Out(vf1_hat));

    EXPECT_EQ(f.t0(), t0);
    EXPECT_EQ(f.tf(), tsplit);
    EXPECT_EQ(f2.t0(), tsplit);
    EXPECT_EQ(f2.tf(), t0 + 0.18);
    DQUAT_CLOSE(xf1_hat, xf1, 1e-3);
    MAT_EQ(vf1_hat, vf1);
}

TEST(ImuFactor, SplitFirst)
{
    UTCTime t(0, 0);
    const UTCTime t0(0, 0);
    const UTCTime tsplit = t0 + 0.01;
    std::vector<meas::ImuSample> imu_samples;
    while (t.toSec() < 0.2)
    {
        meas::ImuSample sample;
        sample.t = t;
        sample.accel = -GRAVITY;
        sample.gyro.setZero();
        imu_samples.push_back(std::move(sample));
        t += 0.02;
    }

    const Vec6 cov = Vec6::Ones() * 1e-3;
    const Vec6 b0 = Vec6::Zero();

    const math::DQuat<double> x0 = math::DQuat<double>::Random();
    const Vec3 v0 = Vec3::Random();

    ImuModel f(imu_samples.front(), b0, cov);
    math::DQuat<double> x_split, xm;
    Vec3 v_split, vm;
    for (size_t i = 1; i < imu_samples.size(); ++i)
    {
        const auto sample = imu_samples[i];
        f.integrate(sample);
    }

    math::DQuat<double> xf1;
    Vec3 vf1;
    f.computeEndState(x0, v0, Out(xf1), Out(vf1));

    ImuModel f2 = f.split(tsplit);

    math::DQuat<double> xm1_hat, xf1_hat;
    Vec3 vm1_hat, vf1_hat;
    f.computeEndState(x0, v0, Out(xm1_hat), Out(vm1_hat));
    f2.computeEndState(xm1_hat, vm1_hat, Out(xf1_hat), Out(vf1_hat));

    EXPECT_EQ(f.t0(), t0);
    EXPECT_EQ(f.tf(), tsplit);
    EXPECT_EQ(f2.t0(), tsplit);
    EXPECT_EQ(f2.tf(), t0 + 0.18);
    DQUAT_CLOSE(xf1_hat, xf1, 1e-3);
    MAT_EQ(vf1_hat, vf1);
}

TEST(ImuFactor, SplitLast)
{
    UTCTime t(0, 0);
    const UTCTime t0(0, 0);
    const UTCTime tsplit = t0 + 0.17;
    std::vector<meas::ImuSample> imu_samples;
    while (t.toSec() < 0.2)
    {
        meas::ImuSample sample;
        sample.t = t;
        sample.accel = -GRAVITY;
        sample.gyro.setZero();
        imu_samples.push_back(std::move(sample));
        t += 0.02;
    }

    const Vec6 cov = Vec6::Ones() * 1e-3;
    const Vec6 b0 = Vec6::Zero();

    const math::DQuat<double> x0 = math::DQuat<double>::Random();
    const Vec3 v0 = Vec3::Random();

    ImuModel f(imu_samples.front(), b0, cov);
    math::DQuat<double> x_split, xm;
    Vec3 v_split, vm;
    for (size_t i = 1; i < imu_samples.size(); ++i)
    {
        const auto sample = imu_samples[i];
        f.integrate(sample);
    }

    math::DQuat<double> xf1;
    Vec3 vf1;
    f.computeEndState(x0, v0, Out(xf1), Out(vf1));

    ImuModel f2 = f.split(tsplit);

    math::DQuat<double> xm1_hat, xf1_hat;
    Vec3 vm1_hat, vf1_hat;
    f.computeEndState(x0, v0, Out(xm1_hat), Out(vm1_hat));
    f2.computeEndState(xm1_hat, vm1_hat, Out(xf1_hat), Out(vf1_hat));

    EXPECT_EQ(f.t0(), t0);
    EXPECT_EQ(f.tf(), tsplit);
    EXPECT_EQ(f2.t0(), tsplit);
    EXPECT_EQ(f2.tf(), t0 + 0.18);
    DQUAT_CLOSE(xf1_hat, xf1, 1e-3);
    MAT_EQ(vf1_hat, vf1);
}

class ImuFactorJacobian : public ::testing::Test
{
 public:
    ImuFactorJacobian() : integrator(meas::ImuSample::Zero(), Vec6::Zero(), R) {}

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
            integrator.integrate(sample);
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
    Vec6 R = Vec6::Ones();
};

TEST_F(ImuFactorJacobian, dRes_dStartPose)
{
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    auto fun = [&](const Vec8& _start_pose) {
        parameters[0] = _start_pose.data();
        Vec9 residuals;
        EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), nullptr));
        return residuals;
    };

    /// TODO:  Think about making this not just at identity
    start_pose = math::DQuat<double>::identity();

    MatRM98 J;
    Vec9 residuals;
    double* jacobians[] = {J.data(), nullptr, nullptr, nullptr, nullptr};
    EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), jacobians));

    Mat98 JFD = compute_jac(Vec8(start_pose.arr_), fun);

    MATRIX_CLOSE(J.rightCols(7), JFD.rightCols(7), 1e-6);
}

TEST_F(ImuFactorJacobian, dRes_dEndPose)
{
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    auto fun = [&](const Vec8& _end_pose) {
        parameters[1] = _end_pose.data();
        Vec9 residuals;
        EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), nullptr));
        return residuals;
    };

    /// TODO:  Think about making this not just at identity
    end_pose = math::DQuat<double>::identity();

    MatRM98 J;
    double* jacobians[] = {nullptr, J.data(), nullptr, nullptr, nullptr};
    Vec9 residuals;
    EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), jacobians));

    Mat98 JFD = compute_jac(Vec8(end_pose.arr_), fun);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

TEST_F(ImuFactorJacobian, dRes_dStartVel)
{
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    auto fun = [&](const Vec3& _start_vel) {
        parameters[2] = _start_vel.data();
        Vec9 residuals;
        EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), nullptr));
        return residuals;
    };

    MatRM93 J;
    double* jacobians[] = {nullptr, nullptr, J.data(), nullptr, nullptr};
    Vec9 residuals;
    EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), jacobians));

    Mat93 JFD = compute_jac(start_vel, fun);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

TEST_F(ImuFactorJacobian, dRes_dEndVel)
{
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    auto fun = [&](const Vec3& _end_vel) {
        parameters[3] = _end_vel.data();

        Vec9 residuals;
        EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), nullptr));
        return residuals;
    };

    MatRM93 J;
    double* jacobians[] = {nullptr, nullptr, nullptr, J.data(), nullptr};
    Vec9 residuals;
    EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), jacobians));

    Mat93 JFD = compute_jac(end_vel, fun);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

TEST_F(ImuFactorJacobian, dAdjustedState_dAdjustment)
{
    auto fun = [this](const Vec9& bias_adjustment) {
        ImuState adjusted_state = integrator.state() + bias_adjustment;
        return adjusted_state;
    };

    ImuErrorState bias_adjustment = integrator.dStatedBias() * bias;

    Mat3 dexp;
    math::Quat<double>::exp(bias_adjustment.gamma, &dexp);

    Mat9 J;
    J.setZero();
    J.block<3, 3>(0, 0).setIdentity();
    J.block<3, 3>(3, 3).setIdentity();
    J.block<3, 3>(6, 6) = dexp.transpose();

    Mat9 JFD = compute_jac(bias_adjustment, fun);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

TEST_F(ImuFactorJacobian, dAdjustedState_dBias)
{
    auto fun = [this](const Vec6& _bias) {
        const Vec9 bias_adjustment = integrator.dStatedBias() * _bias;
        ImuState adjusted_state = integrator.state() + bias_adjustment;
        return adjusted_state;
    };

    ImuErrorState bias_adjustment = integrator.dStatedBias() * bias;

    Mat3 dexp;
    math::Quat<double>::exp(bias_adjustment.gamma, &dexp);

    Mat9 d_adjusted_by_state;
    d_adjusted_by_state.setZero();
    d_adjusted_by_state.block<3, 3>(0, 0).setIdentity();
    d_adjusted_by_state.block<3, 3>(3, 3).setIdentity();
    d_adjusted_by_state.block<3, 3>(6, 6) = dexp.transpose();

    Mat96 J = d_adjusted_by_state * integrator.dStatedBias();

    Mat96 JFD = compute_jac(bias, fun);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

// template <typename Fun, typename... Args>
// Eigen::MatrixXd compute_jac_imu(const ImuState& x, const Fun& fun, const Args&... args)
// {
//     const double eps = 1e-8;
//     Eigen::MatrixXd out;
//     for (int i = 0; i < ImuState::DOF; ++i)
//     {
//         const ImuState xp = x + (ImuErrorState::Unit(i) * eps);
//         const ImuState xm = x + (ImuErrorState::Unit(i) * -eps);
//         const Vec9 ym = fun(xm, args...);
//         const Vec9 yp = fun(xp, args...);

//         if (out.rows() == 0 || out.cols() == 0)
//         {
//             out.resize(decltype(yp - yp)::RowsAtCompileTime, ImuState::DOF);
//         }

//         out.col(i) = (yp - ym) / (2.0 * eps);
//     }
//     return out;
// }

TEST_F(ImuFactorJacobian, dRes_dAdjusted)
{
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

    ImuState adjusted_state;
    adjusted_state.setRandom();

    Mat3 log_jac;
    math::Quat<double> rot_error =
        (adjusted_state.gamma.inverse() * (start_pose.rotation().inverse() * end_pose.rotation()));
    rot_error.log(&log_jac);

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

    const Mat9 JFD = compute_jac(adjusted_state, fun);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

TEST_F(ImuFactorJacobian, dRes_dBias)
{
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    auto fun = [&](const Vec6& _bias) {
        parameters[4] = _bias.data();
        Vec9 residuals;
        EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), nullptr));
        return residuals;
    };

    MatRM96 J;
    double* jacobians[] = {nullptr, nullptr, nullptr, nullptr, J.data()};
    Vec9 residuals;
    EXPECT_TRUE(integrator.Evaluate(parameters, residuals.data(), jacobians));

    Mat96 JFD = compute_jac(bias, fun);

    MATRIX_CLOSE(J, JFD, 1e-6);
}

}  // namespace models
}  // namespace mc
