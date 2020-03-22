#include <gtest/gtest.h>

#include <iostream>

#include "common/test_helpers.h"
#include "core/factors/factors.h"

namespace mc {
namespace factors {

using DQ = math::DQuat<double>;

TEST(ImuFactor, Access)
{
    states::Pose pose1;
    states::Pose pose2;
    states::Velocity vel1;
    states::Velocity vel2;
    states::ClockBias cb;
    states::ImuBias bias;

    pose1.t = UTCTime(12345);
    pose1.pose = DQ::Random();
    pose2.t = UTCTime(3456);
    pose2.pose = DQ::Random();

    vel1.t = UTCTime(67890);
    vel1.vel = Vec3::Random();
    vel2.t = UTCTime(890123);
    vel2.vel = Vec3::Random();

    bias.bias = Vec6::Random();

    const auto factor = Imu(pose1, pose2, vel1, vel2, bias);
    const auto states = factor.base_items();
    EXPECT_EQ(states.size(), 5u);

    EXPECT_EQ(pose1.pose.data(), factor.get<0>().getData());
    EXPECT_EQ(pose2.pose.data(), factor.get<1>().getData());
    EXPECT_EQ(vel1.data(), factor.get<2>().getData());
    EXPECT_EQ(vel2.data(), factor.get<3>().getData());
    EXPECT_EQ(bias.data(), factor.get<4>().getData());

    DQUAT_EQUALS(pose1.pose, factor.pose1.pose);
    EXPECT_EQ(pose1.t, factor.pose1.t);
    DQUAT_EQUALS(pose2.pose, factor.pose2.pose);
    EXPECT_EQ(pose2.t, factor.pose2.t);
    MATRIX_EQUALS(vel1.vel, factor.vel1.vel);
    EXPECT_EQ(vel1.t, factor.vel1.t);
    MATRIX_EQUALS(vel2.vel, factor.vel2.vel);
    EXPECT_EQ(vel2.t, factor.vel2.t);
    MATRIX_EQUALS(bias.bias, factor.imu_bias.bias);

    EXPECT_EQ(pose1.data(), factor.get_base<0>()->data());
    EXPECT_EQ(pose2.data(), factor.get_base<1>()->data());
    EXPECT_EQ(vel1.data(), factor.get_base<2>()->data());
    EXPECT_EQ(vel2.data(), factor.get_base<3>()->data());
    EXPECT_EQ(bias.data(), factor.get_base<4>()->data());

    EXPECT_EQ(pose1.data(), factor.base_items()[0].get().data());
    EXPECT_EQ(pose2.data(), factor.base_items()[1].get().data());
    EXPECT_EQ(vel1.data(), factor.base_items()[2].get().data());
    EXPECT_EQ(vel2.data(), factor.base_items()[3].get().data());
    EXPECT_EQ(bias.data(), factor.base_items()[4].get().data());
}

// TEST(ImuFactor, Modify)
// {
//     states::Pose pose;
//     states::Velocity vel;
//     states::ClockBias cb;
//     states::AntennaPosition ap;

//     pose.t = UTCTime(12345);
//     pose.pose = DQ::Random();

//     vel.t = UTCTime(67890);
//     vel.vel = Vec3::Random();

//     cb.clk = Vec2::Random();
//     ap.p_b2g = Vec3::Random();

//     const auto factor = factors::Gnss(pose, vel, cb, ap);
//     const auto states = factor.base_items();
//     EXPECT_EQ(states.size(), 4u);

//     cb.clk += Vec2::Ones();
//     ap.p_b2g += Vec3::Ones();
//     vel.vel += Vec3::Ones();
//     vel.t += 10.0;
//     pose.pose = pose.pose * DQ::exp(Vec6::Ones() * 0.1);
//     pose.t += 10.0;

//     DQUAT_EQUALS(pose.pose, factor.pose.pose);
//     EXPECT_EQ(pose.t, factor.pose.t);
//     MATRIX_EQUALS(vel.vel, factor.vel.vel);
//     EXPECT_EQ(vel.t, factor.vel.t);
//     MATRIX_EQUALS(cb.clk, factor.clock_bias.clk);
//     MATRIX_EQUALS(ap.p_b2g, factor.antenna_position.p_b2g);
// }

}  // namespace factors
}  // namespace mc
