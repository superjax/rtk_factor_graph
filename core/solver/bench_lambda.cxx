#include <benchmark/benchmark.h>
#include <Eigen/QR>

#include "common/matrix_defs.h"
#include "common/print.h"
#include "common/random.h"
#include "core/solver/lambda.h"

#include <chrono>

namespace mc {
namespace core {
namespace solver {

using namespace Eigen;

static void LTDL(benchmark::State& state)
{
    const int size = state.range(0);
    MatrixXd Q(size, size);
    Q.setRandom();
    const MatrixXd Qahat = Q * Q.transpose();

    MatrixXd L(size, size);
    VectorXd D(size);
    for (auto _ : state)
    {
        LTDL(Qahat, L, D);
    }
}
BENCHMARK(LTDL)->Arg(3)->Arg(6)->Arg(12)->Arg(24)->Arg(36);

static void modfBench(benchmark::State& state)
{
    const int size = state.range(0);
    MatrixXd Q(size, size);
    Q.setRandom();

    for (auto _ : state)
    {
        const MatrixXd A = modf(Q);
        benchmark::DoNotOptimize(A);
    }
}
BENCHMARK(modfBench)->Arg(3)->Arg(6)->Arg(12)->Arg(24)->Arg(36);

static void modfReturnInt(benchmark::State& state)
{
    const int size = state.range(0);
    MatrixXd Q(size, size);
    Q.setRandom();
    MatrixXd int_part(size, size);

    for (auto _ : state)
    {
        const MatrixXd A = modf(Q, &int_part);
        benchmark::DoNotOptimize(A);
    }
}
BENCHMARK(modfReturnInt)->Arg(3)->Arg(6)->Arg(12)->Arg(24)->Arg(36);

static void integerGauss(benchmark::State& state)
{
    const int size = state.range(0);

    MatrixXd L(size, size);
    L.setRandom();
    L *= 10.0;
    L.triangularView<Eigen::Upper>().setZero();
    L.diagonal().setConstant(1.0);
    VectorXd D(size);
    D.setRandom();
    VectorXd a(size);
    a.setRandom();
    MatrixXd Z(size, size);
    Z.setIdentity();

    MatrixXd Ltest = L;
    int i = rand() % (size - 1) + 1;
    int j = rand() % i;

    for (auto _ : state)
    {
        auto start = std::chrono::high_resolution_clock::now();
        integerGaussTransformations(i, j, Ltest, a, Z);
        auto end = std::chrono::high_resolution_clock::now();

        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(end - start);

        state.SetIterationTime(elapsed_seconds.count());

        // L is modified inside of integerGaussTransformations, so reset it here,
        //     and choose a new transformation Ltest = L;
        i = rand() % (size - 1) + 1;
        j = rand() % i;
    }
}
BENCHMARK(integerGauss)->Arg(3)->Arg(6)->Arg(12)->Arg(24)->Arg(36)->UseManualTime();

static void benchPermute(benchmark::State& state)
{
    const int size = state.range(0);

    MatrixXd L(size, size);
    L.setRandom();
    L *= 10.0;
    L.triangularView<Eigen::Upper>().setZero();
    L.diagonal().setConstant(1.0);
    VectorXd D(size);
    D.setRandom();
    VectorXd a(size);
    a.setRandom();
    MatrixXd Z(size, size);
    Z.setIdentity();

    int k = rand() % (size - 1);
    for (auto _ : state)
    {
        const double delta = D(k) + L(k + 1, k) * L(k + 1, k) * D(k + 1);

        auto start = std::chrono::high_resolution_clock::now();
        permute(k, delta, a, L, D, Z);
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_seconds =
            std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        state.SetIterationTime(elapsed_seconds.count());

        k = rand() % (size - 1);
    }
}
BENCHMARK(benchPermute)->Arg(3)->Arg(6)->Arg(12)->Arg(24)->Arg(36)->UseManualTime();

static void benchReduce(benchmark::State& state)
{
    using namespace std::chrono;
    const int size = state.range(0);
    MatrixXd Q(size, size);
    VectorXd ahat(size);

    MatrixXd L(size, size);
    MatrixXd Z(size, size);
    VectorXd zhat(size);
    VectorXd D(size);

    for (auto _ : state)
    {
        Q = randomNormal(size, size);
        MatrixXd Qahat = Q.transpose() * Q;
        ahat.setRandom();

        auto start = high_resolution_clock::now();
        reduction(Qahat, ahat, zhat, L, D, Z);
        auto end = high_resolution_clock::now();
        state.SetIterationTime(duration_cast<duration<double>>(end - start).count());
    }
}
BENCHMARK(benchReduce)->Arg(3)->Arg(6)->Arg(12)->Arg(24)->Arg(36)->UseManualTime();

static void benchLambda(benchmark::State& state)
{
    using namespace std::chrono;
    const int size = state.range(0);

    for (auto _ : state)
    {
        VectorXd ahat = 100 * randomNormal(size);
        MatrixXd A = size * randomNormal(size, size);
        MatrixXd U(size, size);
        U.setIdentity();
        U = A.householderQr().householderQ() * U;
        VectorXd D = randomUniform(0, 1, size).array() + 1e-8;
        MatrixXd Qa = U * D.asDiagonal() * U.transpose();
        VectorXd a_fixed(size);
        double ratio;

        auto start = high_resolution_clock::now();
        lambda(ahat, Qa, a_fixed, Out(ratio));
        auto end = high_resolution_clock::now();
        state.SetIterationTime(duration_cast<duration<double>>(end - start).count());
    }
}
BENCHMARK(benchLambda)->Arg(3)->Arg(6)->Arg(12)->Arg(24)->Arg(36)->UseManualTime();

}  // namespace solver
}  // namespace core
}  // namespace mc
