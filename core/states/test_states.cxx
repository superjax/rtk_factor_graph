#include <gtest/gtest.h>

#include "core/states/states.h"
namespace mc {
namespace states {

TEST(States, BuildStates)
{
    Pose pose;
    Velocity vel;
    AntennaPosition antenna_pos;
    ClockBias clock_bias;
    GnssSwitch gnss_switch;

    // Check dimensions
    EXPECT_EQ(pose.DIM, 8);
    EXPECT_EQ(vel.DIM, 3);
    EXPECT_EQ(antenna_pos.DIM, 3);
    EXPECT_EQ(clock_bias.DIM, 2);
    EXPECT_EQ(gnss_switch.DIM, 1);

    // Ensure that all IDs are unique
    EXPECT_NE(pose.ID, vel.ID);
    EXPECT_NE(pose.ID, antenna_pos.ID);
    EXPECT_NE(pose.ID, clock_bias.ID);
    EXPECT_NE(pose.ID, gnss_switch.ID);
    EXPECT_NE(vel.ID, antenna_pos.ID);
    EXPECT_NE(vel.ID, clock_bias.ID);
    EXPECT_NE(vel.ID, gnss_switch.ID);
    EXPECT_NE(antenna_pos.ID, clock_bias.ID);
    EXPECT_NE(antenna_pos.ID, gnss_switch.ID);
    EXPECT_NE(clock_bias.ID, gnss_switch.ID);
}

TEST(States, CommonInterface)
{
    Velocity vel;
    Pose pose;
    AntennaPosition antenna_pos;
    ClockBias clock_bias;
    GnssSwitch gnss_switch;

    std::vector<State*> states;

    states.push_back(&antenna_pos);
    states.push_back(&pose);
    states.push_back(&vel);
    states.push_back(&clock_bias);
    states.push_back(&gnss_switch);

    EXPECT_EQ(states[0]->ID, AntennaPosition::ID);
    EXPECT_EQ(states[1]->ID, Pose::ID);
    EXPECT_EQ(states[2]->ID, Velocity::ID);
    EXPECT_EQ(states[3]->ID, ClockBias::ID);
    EXPECT_EQ(states[4]->ID, GnssSwitch::ID);

    EXPECT_EQ(states[0]->data(), antenna_pos.data());
    EXPECT_EQ(states[1]->data(), pose.data());
    EXPECT_EQ(states[2]->data(), vel.data());
    EXPECT_EQ(states[3]->data(), clock_bias.data());
    EXPECT_EQ(states[4]->data(), gnss_switch.data());
}

}  // namespace states
}  // namespace mc
