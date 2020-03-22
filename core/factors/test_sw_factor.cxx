#include <gtest/gtest.h>

#include <iostream>

#include "common/test_helpers.h"
#include "core/factors/factors.h"

namespace mc {
namespace factors {

using DQ = math::DQuat<double>;

TEST(SwitchFactor, Access)
{
    states::GnssSwitch sw1;
    states::GnssSwitch sw2;

    sw1.sw = 0.1;
    sw2.sw = 0.2;

    const auto factor = GnssSwitch(sw1, sw2);
    const auto states = factor.base_items();
    EXPECT_EQ(states.size(), 2u);

    EXPECT_EQ(sw1.data(), factor.get<0>().getData());
    EXPECT_EQ(sw2.data(), factor.get<1>().getData());
    EXPECT_EQ(sw1.sw, factor.switch1.sw);
    EXPECT_EQ(sw2.sw, factor.switch2.sw);

    EXPECT_EQ(sw1.data(), factor.get_base<0>()->data());
    EXPECT_EQ(sw2.data(), factor.get_base<1>()->data());

    EXPECT_EQ(sw1.data(), factor.base_items()[0].get().data());
    EXPECT_EQ(sw2.data(), factor.base_items()[1].get().data());
}

TEST(SwitchFactor, Modify)
{
    states::GnssSwitch sw1;
    states::GnssSwitch sw2;

    sw1.sw = 0.1;
    sw2.sw = 0.2;

    const auto factor = GnssSwitch(sw1, sw2);

    sw1.sw += 0.05;
    sw2.sw += 0.05;

    EXPECT_EQ(sw1.sw, factor.switch1.sw);
    EXPECT_EQ(sw2.sw, factor.switch2.sw);
}
}  // namespace factors
}  // namespace mc
