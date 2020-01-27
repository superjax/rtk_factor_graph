#include <benchmark/benchmark.h>

#include "common/math/dquat.h"
#include "common/matrix_defs.h"

namespace mc {
namespace math {

static void passiveTransform(benchmark::State& state)
{
    DQuat<double> Q = DQuat<double>::Random();
    Vec3 v = Vec3::Random();
    for (auto _ : state)
    {
        Vec3 v2 = Q.transformp(v);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(passiveTransform);

static void activeTransform(benchmark::State& state)
{
    DQuat<double> Q = DQuat<double>::Random();
    Vec3 v = Vec3::Random();
    for (auto _ : state)
    {
        Vec3 v2 = Q.transforma(v);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(activeTransform);

static void passiveTransformFloat(benchmark::State& state)
{
    DQuat<float> Q = DQuat<float>::Random();
    Vec3f v = Vec3f::Random();
    for (auto _ : state)
    {
        Vec3f v2 = Q.transformp(v);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(passiveTransformFloat);

static void activeTransformFloat(benchmark::State& state)
{
    DQuat<float> Q = DQuat<float>::Random();
    Vec3f v = Vec3f::Random();
    for (auto _ : state)
    {
        Vec3f v2 = Q.transforma(v);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(activeTransformFloat);

static void concatenate(benchmark::State& state)
{
    DQuat<double> Q1 = DQuat<double>::Random();
    DQuat<double> Q2 = DQuat<double>::Random();
    for (auto _ : state)
    {
        DQuat<double> Q3 = Q1 * Q2;
        benchmark::DoNotOptimize(Q3);
    }
}
BENCHMARK(concatenate);

static void concatenateFloat(benchmark::State& state)
{
    DQuat<float> Q1 = DQuat<float>::Random();
    DQuat<float> Q2 = DQuat<float>::Random();
    for (auto _ : state)
    {
        DQuat<float> Q3 = Q1 * Q2;
        benchmark::DoNotOptimize(Q3);
    }
}
BENCHMARK(concatenateFloat);

static void exp(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized();
    for (auto _ : state)
    {
        DQuat<double> Q = DQuat<double>::exp(v);
        benchmark::DoNotOptimize(Q);
    }
}
BENCHMARK(exp);

static void expTiny(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        DQuat<double> Q = DQuat<double>::exp(v);
        benchmark::DoNotOptimize(Q);
    }
}
BENCHMARK(expTiny);

static void expFloat(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized();
    for (auto _ : state)
    {
        DQuat<float> Q = DQuat<float>::exp(v);
        benchmark::DoNotOptimize(Q);
    }
}
BENCHMARK(expFloat);

static void expTinyFloat(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        DQuat<float> Q = DQuat<float>::exp(v);
        benchmark::DoNotOptimize(Q);
    }
}
BENCHMARK(expTinyFloat);

static void log(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized();
    DQuat<double> Q = DQuat<double>::exp(v);
    for (auto _ : state)
    {
        Vec6 v2 = Q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(log);

static void logTiny(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized() * 1e-8;
    DQuat<double> Q = DQuat<double>::exp(v);
    for (auto _ : state)
    {
        Vec6 v2 = Q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logTiny);

static void logPi(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized() * (M_PI - 1e-6);
    DQuat<double> Q = DQuat<double>::exp(v);
    for (auto _ : state)
    {
        Vec6 v2 = Q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logPi);

static void logFloat(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized();
    DQuat<float> Q = DQuat<float>::exp(v);
    for (auto _ : state)
    {
        Vec6f v2 = Q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logFloat);

static void logTinyFloat(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * 1e-8;
    DQuat<float> Q = DQuat<float>::exp(v);
    for (auto _ : state)
    {
        Vec6f v2 = Q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logTinyFloat);

static void logPiFloat(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * (M_PI - 1e-6);
    DQuat<float> Q = DQuat<float>::exp(v);
    for (auto _ : state)
    {
        Vec6f v2 = Q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logPiFloat);

static void expTinyJac(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        Mat6 jac;
        DQuat<double> Q = DQuat<double>::exp(v, &jac);
        benchmark::DoNotOptimize(Q);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(expTinyJac);

static void expFloatJac(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized();
    for (auto _ : state)
    {
        Mat6f jac;
        DQuat<float> Q = DQuat<float>::exp(v, &jac);
        benchmark::DoNotOptimize(Q);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(expFloatJac);

static void expTinyFloatJac(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        DQuat<float> Q = DQuat<float>::exp(v);
        benchmark::DoNotOptimize(Q);
    }
}
BENCHMARK(expTinyFloatJac);

static void logJac(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized();
    DQuat<double> Q = DQuat<double>::exp(v);
    for (auto _ : state)
    {
        Mat6 jac;
        Vec6 v2 = Q.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logJac);

static void logTinyJac(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized() * 1e-8;
    DQuat<double> Q = DQuat<double>::exp(v);
    for (auto _ : state)
    {
        Mat6 jac;
        Vec6 v2 = Q.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logTinyJac);

static void logPiJac(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized() * (M_PI - 1e-6);
    DQuat<double> Q = DQuat<double>::exp(v);
    for (auto _ : state)
    {
        Mat6 jac;
        Vec6 v2 = Q.log(&jac);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logPiJac);

static void logFloatJac(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized();
    DQuat<float> Q = DQuat<float>::exp(v);
    for (auto _ : state)
    {
        Mat6f jac;
        Vec6f v2 = Q.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logFloatJac);

static void logTinyFloatJac(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * 1e-8;
    DQuat<float> Q = DQuat<float>::exp(v);
    for (auto _ : state)
    {
        Mat6f jac;
        Vec6f v2 = Q.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logTinyFloatJac);

static void logPiFloatJac(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * (M_PI - 1e-6);
    DQuat<float> Q = DQuat<float>::exp(v);
    for (auto _ : state)
    {
        Mat6f jac;
        Vec6f v2 = Q.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logPiFloatJac);

}  // namespace math
}  // namespace mc
