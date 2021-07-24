#include "core/ekf/ekf_gen/example_ekf.h"

#include "gtest/gtest.h"

namespace mc {
namespace ekf {
namespace test {

TestErrorState dynamics(const TestState& x, const TestInput& u, StateJac* dxdx, InputJac* dxdu)
{
    return TestErrorState();
}

template <>
typename vectorMeas::Residual h<vectorMeas>(const typename vectorMeas::ZType& z,
                                            const TestState& x,
                                            typename vectorMeas::Jac* dzdx)
{
    vectorMeas::Residual out;
    return out;
}

template <>
typename varStateMeas::Residual h<varStateMeas>(const typename varStateMeas::ZType& z,
                                                const TestState& x,
                                                typename varStateMeas::Jac* dzdx)
{
    varStateMeas::Residual out;
    return out;
}

template <>
typename variableMeas::Residual h<variableMeas>(const typename variableMeas::ZType& z,
                                                const TestState& x,
                                                typename variableMeas::Jac* dzdx)
{
    variableMeas::Residual out;
    return out;
}

TEST(ExampleEkf, StateSizes)
{
    TestState state;
    EXPECT_TRUE((std::is_same_v<decltype(state.pose), math::DQuat<double>>));
    EXPECT_TRUE((std::is_same_v<decltype(state.pose_se3), math::SE3<double>>));
    EXPECT_TRUE((std::is_same_v<decltype(state.vec3), Vec3>));
    EXPECT_TRUE((std::is_same_v<decltype(state.vec5), Vec5>));
    EXPECT_TRUE((std::is_same_v<decltype(state.variable_quat), std::array<math::Quat<double>, 5>>));
    EXPECT_TRUE((std::is_same_v<decltype(state.variable_vec3), std::array<Vec3, 5>>));
}

// TEST(ExampleEkf, Build)
// {
//     Snapshot snap;

//     ProcessCovariance Qx;
//     InputCovariance Qu;
//     predict<TestEkfType>(snap, UTCTime(0), snap.u, Qx, Qu, make_out(snap));
// }

}  // namespace test
}  // namespace ekf
}  // namespace mc
