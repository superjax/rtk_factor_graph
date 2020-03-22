#include <gtest/gtest.h>

#include <iostream>

#include "common/test_helpers.h"
#include "core/factors/factors.h"

namespace mc {
namespace factors {

using DQ = math::DQuat<double>;

TEST(GnssFactor, Access)
{
    states::Pose pose;
    states::Velocity vel;
    states::ClockBias cb;
    states::AntennaPosition ap;

    pose.t = UTCTime(12345);
    pose.pose = DQ::Random();

    vel.t = UTCTime(67890);
    vel.vel = Vec3::Random();

    cb.clk = Vec2::Random();
    ap.p_b2g = Vec3::Random();

    const auto factor = Gnss(pose, vel, cb, ap);
    const auto states = factor.base_items();
    EXPECT_EQ(states.size(), 4u);

    EXPECT_EQ(pose.pose.data(), factor.get<0>().getData());
    EXPECT_EQ(vel.data(), factor.get<1>().getData());
    EXPECT_EQ(cb.data(), factor.get<2>().getData());
    EXPECT_EQ(ap.data(), factor.get<3>().getData());
    DQUAT_EQUALS(pose.pose, factor.pose.pose);
    EXPECT_EQ(pose.t, factor.pose.t);
    MATRIX_EQUALS(vel.vel, factor.vel.vel);
    EXPECT_EQ(vel.t, factor.vel.t);
    MATRIX_EQUALS(cb.clk, factor.clock_bias.clk);
    MATRIX_EQUALS(ap.p_b2g, factor.antenna_position.p_b2g);

    EXPECT_EQ(pose.pose.data(), factor.get<0>().getData());
    EXPECT_EQ(vel.data(), factor.get_base<1>()->data());
    EXPECT_EQ(cb.data(), factor.get_base<2>()->data());
    EXPECT_EQ(ap.data(), factor.get_base<3>()->data());

    EXPECT_EQ(pose.pose.data(), factor.base_items()[0].get().data());
    EXPECT_EQ(vel.data(), factor.base_items()[1].get().data());
    EXPECT_EQ(cb.data(), factor.base_items()[2].get().data());
    EXPECT_EQ(ap.data(), factor.base_items()[3].get().data());
}

TEST(GnssFactor, Modify)
{
    states::Pose pose;
    states::Velocity vel;
    states::ClockBias cb;
    states::AntennaPosition ap;

    pose.t = UTCTime(12345);
    pose.pose = DQ::Random();

    vel.t = UTCTime(67890);
    vel.vel = Vec3::Random();

    cb.clk = Vec2::Random();
    ap.p_b2g = Vec3::Random();

    const auto factor = factors::Gnss(pose, vel, cb, ap);
    const auto states = factor.base_items();
    EXPECT_EQ(states.size(), 4u);

    cb.clk += Vec2::Ones();
    ap.p_b2g += Vec3::Ones();
    vel.vel += Vec3::Ones();
    vel.t += 10.0;
    pose.pose = pose.pose * DQ::exp(Vec6::Ones() * 0.1);
    pose.t += 10.0;

    DQUAT_EQUALS(pose.pose, factor.pose.pose);
    EXPECT_EQ(pose.t, factor.pose.t);
    MATRIX_EQUALS(vel.vel, factor.vel.vel);
    EXPECT_EQ(vel.t, factor.vel.t);
    MATRIX_EQUALS(cb.clk, factor.clock_bias.clk);
    MATRIX_EQUALS(ap.p_b2g, factor.antenna_position.p_b2g);
}

}  // namespace factors
}  // namespace mc
