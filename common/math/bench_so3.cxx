#include <benchmark/benchmark.h>

#include "common/math/so3.h"
#include "common/matrix_defs.h"

namespace mc {
namespace math {

static void passiveRotate(benchmark::State& state)
{
    SO3<double> R = SO3<double>::Random();
    Vec3 v = Vec3::Random();
    for (auto _ : state)
    {
        Vec3 v2 = R * v;
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(passiveRotate);

static void activeRotate(benchmark::State& state)
{
    SO3<double> R = SO3<double>::Random();
    Vec3 v = Vec3::Random();
    for (auto _ : state)
    {
        Vec3 v2 = R.transpose() * v;
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(activeRotate);

static void passiveRotateFloat(benchmark::State& state)
{
    SO3<float> R = SO3<float>::Random();
    Vec3f v = Vec3f::Random();
    for (auto _ : state)
    {
        Vec3f v2 = R * v;
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(passiveRotateFloat);

static void activeRotateFloat(benchmark::State& state)
{
    SO3<float> R = SO3<float>::Random();
    Vec3f v = Vec3f::Random();
    for (auto _ : state)
    {
        Vec3f v2 = R.transpose() * v;
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(activeRotateFloat);

static void concatenate(benchmark::State& state)
{
    SO3<double> R1 = SO3<double>::Random();
    SO3<double> R2 = SO3<double>::Random();
    for (auto _ : state)
    {
        SO3<double> R3 = R1 * R2;
        benchmark::DoNotOptimize(R3);
    }
}
BENCHMARK(concatenate);

static void concatenateFloat(benchmark::State& state)
{
    SO3<float> R1 = SO3<float>::Random();
    SO3<float> R2 = SO3<float>::Random();
    for (auto _ : state)
    {
        SO3<float> R3 = R1 * R2;
        benchmark::DoNotOptimize(R3);
    }
}
BENCHMARK(concatenateFloat);

static void exp(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized();
    for (auto _ : state)
    {
        SO3<double> R = SO3<double>::exp(v);
        benchmark::DoNotOptimize(R);
    }
}
BENCHMARK(exp);

static void expTiny(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        SO3<double> R = SO3<double>::exp(v);
        benchmark::DoNotOptimize(R);
    }
}
BENCHMARK(expTiny);

static void expFloat(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized();
    for (auto _ : state)
    {
        SO3<float> R = SO3<float>::exp(v);
        benchmark::DoNotOptimize(R);
    }
}
BENCHMARK(expFloat);

static void expTinyFloat(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        SO3<float> R = SO3<float>::exp(v);
        benchmark::DoNotOptimize(R);
    }
}
BENCHMARK(expTinyFloat);

static void log(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized();
    SO3<double> R = SO3<double>::exp(v);
    for (auto _ : state)
    {
        Vec3 v2 = R.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(log);

static void logTiny(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized() * 1e-8;
    SO3<double> R = SO3<double>::exp(v);
    for (auto _ : state)
    {
        Vec3 v2 = R.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logTiny);

static void logPi(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized() * (M_PI - 1e-6);
    SO3<double> R = SO3<double>::exp(v);
    for (auto _ : state)
    {
        Vec3 v2 = R.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logPi);

static void logFloat(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized();
    SO3<float> R = SO3<float>::exp(v);
    for (auto _ : state)
    {
        Vec3f v2 = R.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logFloat);

static void logTinyFloat(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * 1e-8;
    SO3<float> R = SO3<float>::exp(v);
    for (auto _ : state)
    {
        Vec3f v2 = R.log();
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logTinyFloat);

static void logPiFloat(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * (M_PI - 1e-6);
    SO3<float> R = SO3<float>::exp(v);
    for (auto _ : state)
    {
        Vec3f v2 = R.log();
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
        SO3<double> R = SO3<double>::exp(v, &jac);
        benchmark::DoNotOptimize(R);
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
        SO3<float> R = SO3<float>::exp(v, &jac);
        benchmark::DoNotOptimize(R);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(expFloatJac);

static void expTinyFloatJac(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * 1e-6;
    for (auto _ : state)
    {
        SO3<float> R = SO3<float>::exp(v);
        benchmark::DoNotOptimize(R);
    }
}
BENCHMARK(expTinyFloatJac);

static void logJac(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized();
    SO3<double> R = SO3<double>::exp(v);
    for (auto _ : state)
    {
        Mat3 jac;
        Vec3 v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logJac);

static void logTinyJac(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized() * 1e-8;
    SO3<double> R = SO3<double>::exp(v);
    for (auto _ : state)
    {
        Mat3 jac;
        Vec3 v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logTinyJac);

static void logPiJac(benchmark::State& state)
{
    Vec3 v = Vec3::Random().normalized() * (M_PI - 1e-6);
    SO3<double> R = SO3<double>::exp(v);
    for (auto _ : state)
    {
        Mat3 jac;
        Vec3 v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
    }
}
BENCHMARK(logPiJac);

static void logFloatJac(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized();
    SO3<float> R = SO3<float>::exp(v);
    for (auto _ : state)
    {
        Mat3f jac;
        Vec3f v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logFloatJac);

static void logTinyFloatJac(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * 1e-8;
    SO3<float> R = SO3<float>::exp(v);
    for (auto _ : state)
    {
        Mat3f jac;
        Vec3f v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logTinyFloatJac);

static void logPiFloatJac(benchmark::State& state)
{
    Vec3f v = Vec3f::Random().normalized() * (M_PI - 1e-6);
    SO3<float> R = SO3<float>::exp(v);
    for (auto _ : state)
    {
        Mat3f jac;
        Vec3f v2 = R.log(&jac);
        benchmark::DoNotOptimize(v2);
        benchmark::DoNotOptimize(jac);
    }
}
BENCHMARK(logPiFloatJac);

}  // namespace math
}  // namespace mc
