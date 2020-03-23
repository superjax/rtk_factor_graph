#include <benchmark/benchmark.h>

#include "common/math/dquat.h"
#include "common/matrix_defs.h"
#include "models/pose_error_model.h"

namespace mc {
namespace models {

using DQ = math::DQuat<double>;

static void NoJacobians(benchmark::State& state)
{
    const Vec6 Xi = Vec6::Ones();

    const Vec6 perturbation = 0.3 * Vec6::Random();

    const DQ anchor = DQ::Random();
    const DQ pose = anchor * DQ::exp(perturbation);

    PoseError f(anchor, Xi);

    const double* parameters[] = {pose.data()};

    Vec6 residuals;
    for (auto _ : state)
    {
        f.Evaluate(parameters, residuals.data(), nullptr);
    }
}
BENCHMARK(NoJacobians);

static void Jac(benchmark::State& state)
{
    const Vec6 Xi = Vec6::Ones();

    const Vec6 perturbation = 0.3 * Vec6::Random();

    const DQ anchor = DQ::Random();
    const DQ pose = anchor * DQ::exp(perturbation);

    PoseError f(anchor, Xi);

    const double* parameters[] = {pose.data()};

    MatRM68 jac;
    double* jacobians[] = {jac.data()};

    Vec6 residuals;
    for (auto _ : state)
    {
        f.Evaluate(parameters, residuals.data(), jacobians);
    }
}
BENCHMARK(Jac);

}  // namespace models
}  // namespace mc
