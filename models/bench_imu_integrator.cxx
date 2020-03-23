#include <benchmark/benchmark.h>

#include "models/imu_integrator.h"

namespace mc {
namespace models {

static void dynamics(benchmark::State& state)
{
    ImuState imu_state;
    imu_state.setIdentity();
    imu_state = imu_state + ImuErrorState::Random();

    ImuErrorState dstate;
    ImuIntegrator integrator;

    meas::ImuSample imu_sample;
    imu_sample.t = UTCTime::now();
    imu_sample.accel = Vec3::Random();
    imu_sample.gyro = Vec3::Random();

    for (auto _ : state)
    {
        integrator.dynamics(imu_state, imu_sample, Out(dstate));
    }
}
BENCHMARK(dynamics);

static void integrate(benchmark::State& state)
{
    ImuState imu_state;
    imu_state.setIdentity();
    imu_state = imu_state + ImuErrorState::Random();

    ImuIntegrator integrator;

    meas::ImuSample imu_sample;
    imu_sample.t = UTCTime::now();
    imu_sample.accel = Vec3::Random();
    imu_sample.gyro = Vec3::Random();

    for (auto _ : state)
    {
        integrator.integrate(imu_sample);
        imu_sample.t += 0.001;
    }
}
BENCHMARK(integrate);

static void computeEndJet(benchmark::State& state)
{
    ImuState imu_state;
    imu_state.setIdentity();
    imu_state = imu_state + ImuErrorState::Random();

    ImuIntegrator integrator;

    meas::ImuSample imu_sample;
    imu_sample.t = UTCTime::now();
    imu_sample.accel = Vec3::Random();
    imu_sample.gyro = Vec3::Random();

    integrator.reset(imu_sample.t - 0.1);
    integrator.integrate(imu_sample);

    const math::Jet<double> start = math::Jet<double>::Random();
    math::Jet<double> end;
    for (auto _ : state)
    {
        integrator.computeEndJet(start, Out(end));
    }
}
BENCHMARK(computeEndJet);

}  // namespace models
}  // namespace mc
