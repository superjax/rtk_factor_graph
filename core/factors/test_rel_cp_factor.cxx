#include <gtest/gtest.h>

#include <iostream>

#include "common/test_helpers.h"
#include "core/factors/factors.h"

namespace mc {
namespace factors {

using DQ = math::DQuat<double>;

TEST(RelativeCarrierPhaseFactor, Access)
{
    states::Pose pose1;
    states::Pose pose2;
    states::AntennaPosition ap;
    states::ClockBias clk1;
    states::ClockBias clk2;
    states::GnssSwitch sw1;
    states::GnssSwitch sw2;

    pose1.t = UTCTime(12345);
    pose1.pose = DQ::Random();
    pose2.t = UTCTime(67890);
    pose2.pose = DQ::Random();

    ap.p_b2g = Vec3::Random();
    clk1.clk = Vec2::Random();
    clk2.clk = Vec2::Random();
    sw1.sw = 0.234;
    sw2.sw = 0.293;

    const RelativeCarrierPhase factor(pose1, pose2, ap, clk1, clk2, sw1, sw2);
    const auto states = factor.base_items();
    EXPECT_EQ(states.size(), 7u);

    EXPECT_EQ(pose1.data(), factor.get<0>().getData());
    EXPECT_EQ(pose2.data(), factor.get<1>().getData());
    EXPECT_EQ(ap.data(), factor.get<2>().getData());
    EXPECT_EQ(clk1.data(), factor.get<3>().getData());
    EXPECT_EQ(clk2.data(), factor.get<4>().getData());
    EXPECT_EQ(sw1.data(), factor.get<5>().getData());
    EXPECT_EQ(sw2.data(), factor.get<6>().getData());

    DQUAT_EQUALS(pose1.pose, factor.pose1.pose);
    DQUAT_EQUALS(pose2.pose, factor.pose2.pose);
    EXPECT_EQ(pose1.t, factor.pose1.t);
    EXPECT_EQ(pose2.t, factor.pose2.t);
    MATRIX_EQUALS(ap.p_b2g, factor.antenna_position.p_b2g);
    MATRIX_EQUALS(clk1.clk, factor.clock_bias1.clk);
    MATRIX_EQUALS(clk2.clk, factor.clock_bias2.clk);
    EXPECT_EQ(sw1.sw, factor.switch1.sw);
    EXPECT_EQ(sw2.sw, factor.switch2.sw);

    EXPECT_EQ(pose1.data(), factor.get_base<0>()->data());
    EXPECT_EQ(pose2.data(), factor.get_base<1>()->data());
    EXPECT_EQ(ap.data(), factor.get_base<2>()->data());
    EXPECT_EQ(clk1.data(), factor.get_base<3>()->data());
    EXPECT_EQ(clk2.data(), factor.get_base<4>()->data());
    EXPECT_EQ(sw1.data(), factor.get_base<5>()->data());
    EXPECT_EQ(sw2.data(), factor.get_base<6>()->data());

    EXPECT_EQ(pose1.data(), factor.base_items()[0].get().data());
    EXPECT_EQ(pose2.data(), factor.base_items()[1].get().data());
    EXPECT_EQ(ap.data(), factor.base_items()[2].get().data());
    EXPECT_EQ(clk1.data(), factor.base_items()[3].get().data());
    EXPECT_EQ(clk2.data(), factor.base_items()[4].get().data());
    EXPECT_EQ(sw1.data(), factor.base_items()[5].get().data());
    EXPECT_EQ(sw2.data(), factor.base_items()[6].get().data());
}

TEST(RelativeCarrierPhaseFactor, Modify)
{
    states::Pose pose1;
    states::Pose pose2;
    states::AntennaPosition ap;
    states::ClockBias clk1;
    states::ClockBias clk2;
    states::GnssSwitch sw1;
    states::GnssSwitch sw2;

    pose1.t = UTCTime(12345);
    pose1.pose = DQ::Random();
    pose2.t = UTCTime(67890);
    pose2.pose = DQ::Random();

    ap.p_b2g = Vec3::Random();
    clk1.clk = Vec2::Random();
    clk2.clk = Vec2::Random();
    sw1.sw = 0.234;
    sw2.sw = 0.293;

    const RelativeCarrierPhase factor(pose1, pose2, ap, clk1, clk2, sw1, sw2);

    pose1.t += 12.0;
    pose1.pose = DQ::Random();
    pose2.t += 10.0;
    pose2.pose = DQ::Random();

    ap.p_b2g += Vec3::Random();
    clk1.clk += Vec2::Random();
    clk2.clk += Vec2::Random();
    sw1.sw += 0.234;
    sw2.sw += 0.293;

    DQUAT_EQUALS(pose1.pose, factor.pose1.pose);
    DQUAT_EQUALS(pose2.pose, factor.pose2.pose);
    EXPECT_EQ(pose1.t, factor.pose1.t);
    EXPECT_EQ(pose2.t, factor.pose2.t);
    MATRIX_EQUALS(ap.p_b2g, factor.antenna_position.p_b2g);
    MATRIX_EQUALS(clk1.clk, factor.clock_bias1.clk);
    MATRIX_EQUALS(clk2.clk, factor.clock_bias2.clk);
    EXPECT_EQ(sw1.sw, factor.switch1.sw);
    EXPECT_EQ(sw2.sw, factor.switch2.sw);
}

}  // namespace factors
}  // namespace mc
