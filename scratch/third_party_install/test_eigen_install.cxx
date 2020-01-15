#include "Eigen/Core"

#include "gtest/gtest.h"

TEST(Eigen, InstalledProperly)
{
    Eigen::Vector3d test(1, 2, 3);
    EXPECT_FLOAT_EQ(test[0], 1);
    EXPECT_FLOAT_EQ(test[0], 1);
    EXPECT_FLOAT_EQ(test[0], 1);
}
