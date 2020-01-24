#include <benchmark/benchmark.h>

#include "common/math/se3.h"
#include "common/matrix_defs.h"

static void passiveTransform(benchmark::State& state)
{
    SE3<double> R = SE3<double>::Random();
    Vec3 v = Vec3::Random();
    for (auto _ : state)
    {
        Vec3 v2 = R.transformp(v);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(passiveTransform);

static void activeTransform(benchmark::State& state)
{
    SE3<double> R = SE3<double>::Random();
    Vec3 v = Vec3::Random();
    for (auto _ : state)
    {
        Vec3 v2 = R.transforma(v);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(activeTransform);

static void passiveTransformFloat(benchmark::State& state)
{
    SE3<float> R = SE3<float>::Random();
    Vec3f v = Vec3f::Random();
    for (auto _ : state)
    {
        Vec3f v2 = R.transformp(v);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(passiveTransformFloat);

static void activeTransformFloat(benchmark::State& state)
{
    SE3<float> R = SE3<float>::Random();
    Vec3f v = Vec3f::Random();
    for (auto _ : state)
    {
        Vec3f v2 = R.transforma(v);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(activeTransformFloat);

static void concatenate(benchmark::State& state)
{
    SE3<double> R1 = SE3<double>::Random();
    SE3<double> R2 = SE3<double>::Random();
    for (auto _ : state)
    {
        SE3<double> R3 = R1 * R2;
        benchmark::DoNotOptimize(R3);
    }
}
BENCHMARK(concatenate);

static void concatenateFloat(benchmark::State& state)
{
    SE3<float> R1 = SE3<float>::Random();
    SE3<float> R2 = SE3<float>::Random();
    for (auto _ : state)
    {
        SE3<float> R3 = R1 * R2;
        benchmark::DoNotOptimize(R3);
    }
}
BENCHMARK(concatenateFloat);

static void exp(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized();
    for (auto _ : state)
    {
        SE3<double> R = SE3<double>::exp(v);
        benchmark::DoNotOptimize(R);
    }
}
BENCHMARK(exp);

static void expTiny(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        SE3<double> R = SE3<double>::exp(v);
        benchmark::DoNotOptimize(R);
    }
}
BENCHMARK(expTiny);

static void expFloat(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized();
    for (auto _ : state)
    {
        SE3<float> R = SE3<float>::exp(v);
        benchmark::DoNotOptimize(R);
    }
}
BENCHMARK(expFloat);

static void expTinyFloat(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        SE3<float> R = SE3<float>::exp(v);
        benchmark::DoNotOptimize(R);
    }
}
BENCHMARK(expTinyFloat);

static void log(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized();
    SE3<double> R = SE3<double>::exp(v);
    for (auto _ : state)
    {
        Vec6 v2 = R.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(log);

static void logTiny(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized() * 1e-8;
    SE3<double> R = SE3<double>::exp(v);
    for (auto _ : state)
    {
        Vec6 v2 = R.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logTiny);

static void logPi(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized() * (M_PI - 1e-6);
    SE3<double> R = SE3<double>::exp(v);
    for (auto _ : state)
    {
        Vec6 v2 = R.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logPi);

static void logFloat(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized();
    SE3<float> R = SE3<float>::exp(v);
    for (auto _ : state)
    {
        Vec6f v2 = R.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logFloat);

static void logTinyFloat(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * 1e-8;
    SE3<float> R = SE3<float>::exp(v);
    for (auto _ : state)
    {
        Vec6f v2 = R.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logTinyFloat);

static void logPiFloat(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * (M_PI - 1e-6);
    SE3<float> R = SE3<float>::exp(v);
    for (auto _ : state)
    {
        Vec6f v2 = R.log();
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
        SE3<double> R = SE3<double>::exp(v, &jac);
        benchmark::DoNotOptimize(R);
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
        SE3<float> R = SE3<float>::exp(v, &jac);
        benchmark::DoNotOptimize(R);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(expFloatJac);

static void expTinyFloatJac(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        SE3<float> R = SE3<float>::exp(v);
        benchmark::DoNotOptimize(R);
    }
}
BENCHMARK(expTinyFloatJac);

static void logJac(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized();
    SE3<double> R = SE3<double>::exp(v);
    for (auto _ : state)
    {
        Mat6 jac;
        Vec6 v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logJac);

static void logTinyJac(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized() * 1e-8;
    SE3<double> R = SE3<double>::exp(v);
    for (auto _ : state)
    {
        Mat6 jac;
        Vec6 v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logTinyJac);

static void logPiJac(benchmark::State& state)
{
    Vec6 v = Vec6::Random().normalized() * (M_PI - 1e-6);
    SE3<double> R = SE3<double>::exp(v);
    for (auto _ : state)
    {
        Mat6 jac;
        Vec6 v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logPiJac);

static void logFloatJac(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized();
    SE3<float> R = SE3<float>::exp(v);
    for (auto _ : state)
    {
        Mat6f jac;
        Vec6f v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logFloatJac);

static void logTinyFloatJac(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * 1e-8;
    SE3<float> R = SE3<float>::exp(v);
    for (auto _ : state)
    {
        Mat6f jac;
        Vec6f v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logTinyFloatJac);

static void logPiFloatJac(benchmark::State& state)
{
    Vec6f v = Vec6f::Random().normalized() * (M_PI - 1e-6);
    SE3<float> R = SE3<float>::exp(v);
    for (auto _ : state)
    {
        Mat6f jac;
        Vec6f v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logPiFloatJac);
