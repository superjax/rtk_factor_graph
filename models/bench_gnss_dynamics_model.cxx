#include <benchmark/benchmark.h>

#include "common/ephemeris/gps.h"
#include "common/matrix_defs.h"
#include "common/satellite/satellite.h"
#include "models/gnss_dynamics_model.h"

namespace mc {
namespace models {

class TestGnssDynamics : public benchmark::Fixture
{
 public:
    TestGnssDynamics()
    {
        t1 = UTCTime(12345);
        const double dt = 0.2;
        t2 = t1 + dt;

        const double clock_bias1 = 0.0000235 * 1e9;
        const double clock_bias_rate1 = 0.000012934 * 1e9;
        const double clock_bias_rate2 = 0.000037182 * 1e9;
        const double avg_clock_bias_rate = (clock_bias_rate1 + clock_bias_rate2) / 2.0;
        const double clock_bias2 = clock_bias1 + avg_clock_bias_rate * dt + 4;

        sw1 = 0.192;
        sw2 = 0.193;

        clk1 = Vec2(clock_bias1, clock_bias_rate1);
        clk2 = Vec2(clock_bias2, clock_bias_rate2);
    }

    UTCTime t1;
    UTCTime t2;

    Vec2 clk1;
    Vec2 clk2;

    double sw1 = 0.192;
    double sw2 = 0.193;

    Vec3 Xi = Vec3::Ones();
};

BENCHMARK_F(TestGnssDynamics, EvaluateNoJac)(benchmark::State& st)
{
    const GnssDynamics f(t1, t2, Xi);
    const double* parameters[4] = {clk1.data(), clk2.data(), &sw1, &sw2};
    Vec3 residuals;

    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), nullptr);
    }
}

BENCHMARK_F(TestGnssDynamics, Clk1Jac)(benchmark::State& st)
{
    const GnssDynamics f(t1, t2, Xi);
    const double* parameters[4] = {clk1.data(), clk2.data(), &sw1, &sw2};
    MatRM32 jac;
    double* jacobians[] = {jac.data(), nullptr, nullptr, nullptr};
    Vec3 residuals;

    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(TestGnssDynamics, Clk2Jac)(benchmark::State& st)
{
    const GnssDynamics f(t1, t2, Xi);
    const double* parameters[4] = {clk1.data(), clk2.data(), &sw1, &sw2};
    MatRM32 jac;
    double* jacobians[] = {nullptr, jac.data(), nullptr, nullptr};
    Vec3 residuals;

    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(TestGnssDynamics, Sw1Jac)(benchmark::State& st)
{
    const GnssDynamics f(t1, t2, Xi);
    const double* parameters[4] = {clk1.data(), clk2.data(), &sw1, &sw2};
    Vec3 jac;
    double* jacobians[] = {nullptr, nullptr, jac.data(), nullptr};
    Vec3 residuals;

    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}

BENCHMARK_F(TestGnssDynamics, Sw2Jac)(benchmark::State& st)
{
    const GnssDynamics f(t1, t2, Xi);
    const double* parameters[4] = {clk1.data(), clk2.data(), &sw1, &sw2};
    Vec3 jac;
    double* jacobians[] = {nullptr, nullptr, nullptr, jac.data()};
    Vec3 residuals;

    for (auto _ : st)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}

}  // namespace models
}  // namespace mc
