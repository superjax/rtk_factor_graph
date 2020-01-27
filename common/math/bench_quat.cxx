#include <benchmark/benchmark.h>

#include "common/math/quat.h"
#include "common/matrix_defs.h"

namespace mc {
namespace math {

static void passiveRotate(benchmark::State& state)
{
    Quat<double> q = Quat<double>::Random();
    Vec3 v = Vec3::Random();
    for (auto _ : state)
    {
        Vec3 v2 = q.rotp(v);
        benchmark::DoNotOptimize(v2);
        (void)v2;
    }
}
BENCHMARK(passiveRotate);

static void activeRotate(benchmark::State& state)
{
    Quat<double> q = Quat<double>::Random();
    Vec3 v = Vec3::Random();
    for (auto _ : state)
    {
        Vec3 v2 = q.rota(v);
        benchmark::DoNotOptimize(v2);
        (void)v2;
    }
}
BENCHMARK(activeRotate);

static void passiveRotateFloat(benchmark::State& state)
{
    Quat<float> q = Quat<float>::Random();
    Vec3f v = Vec3f::Random();
    for (auto _ : state)
    {
        Vec3f v2 = q.rotp(v);
        benchmark::DoNotOptimize(v2);
        (void)v2;
    }
}
BENCHMARK(passiveRotateFloat);

static void activeRotateFloat(benchmark::State& state)
{
    Quat<float> q = Quat<float>::Random();
    Vec3f v = Vec3f::Random();
    for (auto _ : state)
    {
        Vec3f v2 = q.rota(v);
        benchmark::DoNotOptimize(v2);
        (void)v2;
    }
}
BENCHMARK(activeRotateFloat);

static void otimesRotateFloat(benchmark::State& state)
{
    Quat<float> q = Quat<float>::Random();
    Quat<float> v = Quat<float>::make_pure(Vec3f::Random());
    for (auto _ : state)
    {
        Vec3f v2 = (q * v * q.inverse()).bar();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(otimesRotateFloat);

static void concatenate(benchmark::State& state)
{
    Quat<double> q1 = Quat<double>::Random();
    Quat<double> q2 = Quat<double>::Random();
    for (auto _ : state)
    {
        Quat<double> q3 = q1 * q2;
        benchmark::DoNotOptimize(q3);
    }
}
BENCHMARK(concatenate);

static void concatenateFloat(benchmark::State& state)
{
    Quat<float> q1 = Quat<float>::Random();
    Quat<float> q2 = Quat<float>::Random();
    for (auto _ : state)
    {
        Quat<float> q3 = q1 * q2;
        benchmark::DoNotOptimize(q3);
    }
}
BENCHMARK(concatenateFloat);

static void exp(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized();
    for (auto _ : state)
    {
        Quat<double> q = Quat<double>::exp(v);
        benchmark::DoNotOptimize(q);
    }
}
BENCHMARK(exp);

static void expTiny(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        Quat<double> q = Quat<double>::exp(v);
        benchmark::DoNotOptimize(q);
    }
}
BENCHMARK(expTiny);

static void expFloat(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized();
    for (auto _ : state)
    {
        Quat<float> q = Quat<float>::exp(v);
        benchmark::DoNotOptimize(q);
    }
}
BENCHMARK(expFloat);

static void expTinyFloat(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        Quat<float> q = Quat<float>::exp(v);
        benchmark::DoNotOptimize(q);
    }
}
BENCHMARK(expTinyFloat);

static void log(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized();
    Quat<double> q = Quat<double>::exp(v);
    for (auto _ : state)
    {
        Vec3 v2 = q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(log);

static void logTiny(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized() * 1e-8;
    Quat<double> q = Quat<double>::exp(v);
    for (auto _ : state)
    {
        Vec3 v2 = q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logTiny);

static void logPi(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized() * (M_PI - 1e-6);
    Quat<double> q = Quat<double>::exp(v);
    for (auto _ : state)
    {
        Vec3 v2 = q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logPi);

static void logFloat(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized();
    Quat<float> q = Quat<float>::exp(v);
    for (auto _ : state)
    {
        Vec3f v2 = q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logFloat);

static void logTinyFloat(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * 1e-8;
    Quat<float> q = Quat<float>::exp(v);
    for (auto _ : state)
    {
        Vec3f v2 = q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logTinyFloat);

static void logPiFloat(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * (M_PI - 1e-6);
    Quat<float> q = Quat<float>::exp(v);
    for (auto _ : state)
    {
        Vec3f v2 = q.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logPiFloat);

static void expTinyJac(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        Mat3 jac;
        Quat<double> q = Quat<double>::exp(v, &jac);
        benchmark::DoNotOptimize(q);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(expTinyJac);

static void expFloatJac(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized();
    for (auto _ : state)
    {
        Mat3f jac;
        Quat<float> q = Quat<float>::exp(v, &jac);
        benchmark::DoNotOptimize(q);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(expFloatJac);

static void expTinyFloatJac(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        Quat<float> q = Quat<float>::exp(v);
        benchmark::DoNotOptimize(q);
    }
}
BENCHMARK(expTinyFloatJac);

static void logJac(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized();
    Quat<double> q = Quat<double>::exp(v);
    for (auto _ : state)
    {
        Mat3 jac;
        Vec3 v2 = q.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logJac);

static void logTinyJac(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized() * 1e-8;
    Quat<double> q = Quat<double>::exp(v);
    for (auto _ : state)
    {
        Mat3 jac;
        Vec3 v2 = q.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logTinyJac);

static void logPiJac(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized() * (M_PI - 1e-6);
    Quat<double> q = Quat<double>::exp(v);
    for (auto _ : state)
    {
        Mat3 jac;
        Vec3 v2 = q.log(&jac);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logPiJac);

static void logFloatJac(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized();
    Quat<float> q = Quat<float>::exp(v);
    for (auto _ : state)
    {
        Mat3f jac;
        Vec3f v2 = q.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logFloatJac);

static void logTinyFloatJac(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * 1e-8;
    Quat<float> q = Quat<float>::exp(v);
    for (auto _ : state)
    {
        Mat3f jac;
        Vec3f v2 = q.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logTinyFloatJac);

static void logPiFloatJac(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * (M_PI - 1e-6);
    Quat<float> q = Quat<float>::exp(v);
    for (auto _ : state)
    {
        Mat3f jac;
        Vec3f v2 = q.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logPiFloatJac);

}  // namespace math
}  // namespace mc
