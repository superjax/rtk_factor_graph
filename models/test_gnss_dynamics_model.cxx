#include <gtest/gtest.h>

#include "common/numerical_jacobian.h"
#include "common/test_helpers.h"
#include "common/utctime.h"
#include "models/gnss_dynamics_model.h"

namespace mc {
namespace models {

class TestGnssDynamics : public ::testing::Test
{
 public:
    void SetUp() override
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

TEST_F(TestGnssDynamics, HandComputed)
{
    const GnssDynamics f(t1, t2, Xi);
    const double* parameters[4] = {clk1.data(), clk2.data(), &sw1, &sw2};
    Vec3 residuals;

    EXPECT_TRUE(f.Evaluate(parameters, residuals.data(), nullptr));

    MATRIX_EQUALS(residuals, Vec3(4, 24248, 0.001));
}

TEST_F(TestGnssDynamics, Clk1Jacobian)
{
    const GnssDynamics f(t1, t2, Xi);

    const auto fun = [this, f](const Vec2& _clk1) -> Vec3 {
        const double* parameters[4] = {_clk1.data(), clk2.data(), &sw1, &sw2};
        Vec3 residuals;
        f.Evaluate(parameters, residuals.data(), nullptr);
        return residuals;
    };

    const double* parameters[4] = {clk1.data(), clk2.data(), &sw1, &sw2};
    Vec3 residuals;
    MatRM32 jac;
    double* jacobians[] = {jac.data(), nullptr, nullptr, nullptr};
    EXPECT_TRUE(f.Evaluate(parameters, residuals.data(), jacobians));

    MatRM32 num_jac = compute_jac(clk1, fun, 1e-6);

    MATRIX_CLOSE(jac, num_jac, 1e-5);
}

TEST_F(TestGnssDynamics, Clk2Jacobian)
{
    const GnssDynamics f(t1, t2, Xi);

    const auto fun = [this, f](const Vec2& _clk2) -> Vec3 {
        const double* parameters[4] = {clk1.data(), _clk2.data(), &sw1, &sw2};
        Vec3 residuals;
        f.Evaluate(parameters, residuals.data(), nullptr);
        return residuals;
    };

    const double* parameters[4] = {clk1.data(), clk2.data(), &sw1, &sw2};
    Vec3 residuals;
    MatRM32 jac;
    double* jacobians[] = {nullptr, jac.data(), nullptr, nullptr};
    EXPECT_TRUE(f.Evaluate(parameters, residuals.data(), jacobians));

    MatRM32 num_jac = compute_jac(clk2, fun, 1e-6);

    MATRIX_CLOSE(jac, num_jac, 1e-5);
}

TEST_F(TestGnssDynamics, Sw1Jacobian)
{
    const GnssDynamics f(t1, t2, Xi);

    const auto fun = [this, f](const Vec1 _sw1) -> Vec3 {
        const double* parameters[4] = {clk1.data(), clk2.data(), _sw1.data(), &sw2};
        Vec3 residuals;
        f.Evaluate(parameters, residuals.data(), nullptr);
        return residuals;
    };

    const double* parameters[4] = {clk1.data(), clk2.data(), &sw1, &sw2};
    Vec3 residuals;
    Vec3 jac;
    double* jacobians[] = {nullptr, nullptr, jac.data(), nullptr};
    EXPECT_TRUE(f.Evaluate(parameters, residuals.data(), jacobians));

    Vec3 num_jac = compute_jac(Vec1(sw1), fun, 1e-6);

    MATRIX_CLOSE(jac, num_jac, 1e-5);
}

TEST_F(TestGnssDynamics, Sw2Jacobian)
{
    const GnssDynamics f(t1, t2, Xi);

    const auto fun = [this, f](const Vec1 _sw2) -> Vec3 {
        const double* parameters[4] = {clk1.data(), clk2.data(), &sw1, _sw2.data()};
        Vec3 residuals;
        f.Evaluate(parameters, residuals.data(), nullptr);
        return residuals;
    };

    const double* parameters[4] = {clk1.data(), clk2.data(), &sw1, &sw2};
    Vec3 residuals;
    Vec3 jac;
    double* jacobians[] = {nullptr, nullptr, nullptr, jac.data()};
    EXPECT_TRUE(f.Evaluate(parameters, residuals.data(), jacobians));

    Vec3 num_jac = compute_jac(Vec1(sw2), fun, 1e-6);

    MATRIX_CLOSE(jac, num_jac, 1e-5);
}

}  // namespace models
}  // namespace mc
