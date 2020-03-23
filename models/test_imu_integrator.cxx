#include <gtest/gtest.h>

#include "common/math/jet.h"
#include "common/test_helpers.h"
#include "models/imu_integrator.h"

namespace mc {
namespace models {

TEST(ImuIntegrator, Construct)
{
    ImuIntegrator imu;
    EXPECT_EQ(imu.t0(), INVALID_TIME);
    EXPECT_EQ(imu.tf(), INVALID_TIME);
    EXPECT_TRUE(imu.accelBias().isZero());
    EXPECT_TRUE(imu.gyroBias().isZero());
    EXPECT_TRUE(imu.state().alpha.isZero());
    EXPECT_TRUE(imu.state().beta.isZero());
    QUATERNION_EQUALS(imu.state().gamma, math::Quat<double>::Identity());
}

TEST(ImuIntegrator, integrateZeroImu)
{
    math::Jet<double> x = math::Jet<double>::Identity();
    meas::ImuSample sample;
    sample.setZero();
    sample.accel.z() = -9.80665;

    ImuIntegrator integrator;
    integrator.reset(sample.t);

    constexpr double dt = 0.0001;
    for (int i = 0; i < 10000; ++i)
    {
        sample.t += dt;
        integrator.integrate(sample);
    }

    math::Jet<double> x2;
    integrator.computeEndJet(x, Out(x2));

    QUATERNION_EQUALS(x2.x.real(), x.x.real());
    // Numerical error from Euler's method in ImuIntegrator::integrate
    MATRIX_CLOSE(x2.x.dual().arr_, x.x.dual().arr_, 1e-3);
    MATRIX_EQUALS(x.linear_vel, Vec3::Zero());
    MATRIX_EQUALS(x.angular_vel, Vec3::Zero());
}

TEST(ImuIntegrator, integrateZeroAccel)
{
    math::Jet<double> x = math::Jet<double>::Identity();
    meas::ImuSample sample;
    sample.setZero();

    ImuIntegrator integrator;
    integrator.reset(sample.t);

    const Vec3 gravity(0, 0, 9.80665);
    const Vec3 omega = Vec3::Random();
    math::Quat<double> q = math::Quat<double>::Identity();

    constexpr double dt = 0.0001;
    for (int i = 0; i < 10000; ++i)
    {
        sample.t += dt;
        sample.accel = q.rotp(-gravity);
        sample.gyro = omega;
        integrator.integrate(sample);
        q = q * math::Quat<double>::exp(omega * dt);
    }

    math::Jet<double> x2;
    integrator.computeEndJet(x, Out(x2));

    QUATERNION_EQUALS(x2.x.rotation(), q);
    // Numerical error from Euler's method in ImuIntegrator::integrate
    MATRIX_CLOSE(x2.x.translation(), Vec3::Zero(), 1e-3);
    MATRIX_EQUALS(x2.linear_vel, Vec3::Zero());
    MATRIX_EQUALS(x2.angular_vel, omega);
}

TEST(ImuIntegrator, integrateZeroTime)
{
    math::Jet<double> x = math::Jet<double>::Random();
    meas::ImuSample sample;
    sample.setRandom();

    ImuIntegrator integrator;
    integrator.reset(sample.t);

    integrator.integrate(sample);

    math::Jet<double> x2;
    integrator.computeEndJet(x, Out(x2));

    DQUAT_EQUALS(x2.x, x.x);
    MATRIX_EQUALS(x2.linear_vel, x.linear_vel);
    MATRIX_EQUALS(x2.angular_vel, x.angular_vel);
}

TEST(ImuIntegrator, integrateZeroGyro)
{
    math::Jet<double> x = math::Jet<double>::Identity();
    meas::ImuSample sample;
    sample.setZero();

    ImuIntegrator integrator;
    integrator.reset(sample.t);

    const Vec3 gravity(0, 0, 9.80665);
    const Vec3 accel = -gravity;

    constexpr double dt = 0.0001;
    for (int i = 0; i < 10000; ++i)
    {
        sample.t += dt;
        sample.gyro = Vec3::Zero();
        sample.accel = accel;
        integrator.integrate(sample);
    }

    math::Jet<double> x2;
    integrator.computeEndJet(x, Out(x2));

    QUATERNION_EQUALS(x.x.rotation(), x2.x.rotation());
    // Numerical error from Euler's method in ImuIntegrator::integrate
    const double t = integrator.dt();
    MATRIX_CLOSE(x2.x.translation(), 0.5 * (accel + gravity) * t * t, 1e-3);
    MATRIX_EQUALS(x2.linear_vel, (accel + gravity) * t);
    MATRIX_EQUALS(x2.angular_vel, Vec3::Zero());
}

TEST(ImuIntegrator, integrateRandomImu)
{
    math::Jet<double> x1 = math::Jet<double>::Identity();
    math::Jet<double> x = x1;
    meas::ImuSample sample;
    sample.setZero();

    ImuIntegrator integrator;
    integrator.reset(sample.t);

    const Vec3 gravity(0, 0, 9.80665);

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
        x.x = x.x * math::DQuat<double>::exp(vstack(omega * dt, x.linear_vel * dt));
        x.linear_vel += dt * (accel + x.x.rotation().rotp(gravity));
        x.angular_vel = omega;
        integrator.integrate(sample);
    }

    math::Jet<double> x2;
    integrator.computeEndJet(x1, Out(x2));

    QUATERNION_EQUALS(x.x.rotation(), x2.x.rotation());
    // Numerical error from Euler's method in ImuIntegrator::integrate
    MATRIX_CLOSE(x.x.translation(), x2.x.translation(), 1e-3);
    MATRIX_CLOSE(x.linear_vel, x2.linear_vel, 1e-4);
    MATRIX_EQUALS(x.angular_vel, x2.angular_vel);
}

}  // namespace models
}  // namespace mc
