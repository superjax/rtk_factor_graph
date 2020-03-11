#include <benchmark/benchmark.h>

#include "common/matrix_defs.h"
#include "factors/imu_functor.h"

namespace mc {
namespace factors {

static void dynamics(benchmark::State& state)
{
    ImuState imu_state;
    imu_state.setIdentity();
    imu_state = imu_state + ImuErrorState::Random();

    ImuErrorState dstate;
    ImuFunctor integrator(UTCTime(0, 0), Vec6::Zero());

    meas::ImuSample imu_sample;
    imu_sample.t = UTCTime::now();
    imu_sample.accel = Vec3::Random();
    imu_sample.gyro = Vec3::Random();

    Mat9 A;
    Mat96 B;

    for (auto _ : state)
    {
        integrator.dynamics(imu_state, imu_sample, Out(dstate), Out(A), Out(B));
    }
}
BENCHMARK(dynamics);

static void integrate(benchmark::State& state)
{
    ImuState imu_state;
    imu_state.setIdentity();
    imu_state = imu_state + ImuErrorState::Random();

    ImuFunctor integrator(UTCTime(0, 0), Vec6::Zero());
    const Mat6 cov = Mat6::Identity();

    meas::ImuSample imu_sample;
    imu_sample.t = UTCTime::now();
    imu_sample.accel = Vec3::Random();
    imu_sample.gyro = Vec3::Random();

    for (auto _ : state)
    {
        integrator.integrate(imu_sample, cov);
        imu_sample.t += 0.001;
    }
}
BENCHMARK(integrate);

static void computeEndJet(benchmark::State& state)
{
    ImuState imu_state;
    imu_state.setIdentity();
    imu_state = imu_state + ImuErrorState::Random();

    ImuFunctor integrator(UTCTime(0, 0), Vec6::Zero());
    const Mat6 cov = Mat6::Identity();

    meas::ImuSample imu_sample;
    imu_sample.t = UTCTime::now();
    imu_sample.accel = Vec3::Random();
    imu_sample.gyro = Vec3::Random();

    integrator.reset(imu_sample.t - 0.1);
    integrator.integrate(imu_sample, cov);

    const math::Jet<double> start = math::Jet<double>::Random();
    math::Jet<double> end;
    for (auto _ : state)
    {
        integrator.computeEndJet(start, Out(end));
    }
}
BENCHMARK(computeEndJet);

class Evaluate : public benchmark::Fixture
{
 public:
    Evaluate() : integrator(UTCTime(0, 0), Vec6::Zero())
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

    ImuFunctor integrator;
    math::DQuat<double> start_pose;
    math::DQuat<double> end_pose;
    Vec3 start_vel;
    Vec3 end_vel;
    Vec6 bias;
};

BENCHMARK_F(Evaluate, NoJac)(benchmark::State& st)
{
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    Vec9 residuals;

    for (auto _ : st)
    {
        integrator.Evaluate(parameters, residuals.data(), nullptr);
    }
}

BENCHMARK_F(Evaluate, StartPose)(benchmark::State& st)
{
    MatRM98 J;
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    double* jacobians[] = {J.data(), nullptr, nullptr, nullptr, nullptr};
    Vec9 residuals;

    for (auto _ : st)
    {
        integrator.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(Evaluate, EndPose)(benchmark::State& st)
{
    MatRM98 J;
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    double* jacobians[] = {nullptr, J.data(), nullptr, nullptr, nullptr};
    Vec9 residuals;

    for (auto _ : st)
    {
        integrator.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(Evaluate, StartVel)(benchmark::State& st)
{
    MatRM93 J;
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    double* jacobians[] = {nullptr, nullptr, J.data(), nullptr, nullptr};
    Vec9 residuals;

    for (auto _ : st)
    {
        integrator.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(Evaluate, EndVel)(benchmark::State& st)
{
    MatRM93 J;
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    double* jacobians[] = {nullptr, nullptr, nullptr, J.data(), nullptr};
    Vec9 residuals;

    for (auto _ : st)
    {
        integrator.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(Evaluate, Bias)(benchmark::State& st)
{
    MatRM96 J;
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    double* jacobians[] = {nullptr, nullptr, nullptr, nullptr, J.data()};
    Vec9 residuals;

    for (auto _ : st)
    {
        integrator.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(Evaluate, All)(benchmark::State& st)
{
    MatRM98 J1, J2;
    MatRM93 J3, J4;
    MatRM96 J5;
    const double* parameters[] = {start_pose.data(), end_pose.data(), start_vel.data(),
                                  end_vel.data(), bias.data()};
    double* jacobians[] = {J1.data(), J2.data(), J3.data(), J4.data(), J5.data()};
    Vec9 residuals;

    for (auto _ : st)
    {
        integrator.Evaluate(parameters, residuals.data(), jacobians);
    }
}

}  // namespace factors
}  // namespace mc
