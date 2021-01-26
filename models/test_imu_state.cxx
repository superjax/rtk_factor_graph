#include <gtest/gtest.h>

#include "models/imu_state.h"

namespace mc {
namespace models {

TEST(ImuState, CopyConstructor)
{
    Vec10 x_vec;
    x_vec.setRandom();
    ImuState x(x_vec);

    for (int i = 0; i < 10; ++i)
    {
        EXPECT_DOUBLE_EQ(x(i), x_vec(i));
    }
}

TEST(ImuState, EqualsVector)
{
    Vec10 x_vec;
    x_vec.setRandom();
    ImuState x;
    x = x_vec;

    for (int i = 0; i < 10; ++i)
    {
        EXPECT_DOUBLE_EQ(x(i), x_vec(i));
    }
}

TEST(ImuErrorState, CopyConstructorVec)
{
    Vec9 x_vec = Vec9::Random();
    ImuErrorState x(x_vec);

    for (int i = 0; i < 9; ++i)
    {
        EXPECT_DOUBLE_EQ(x(i), x_vec(i));
    }
}

TEST(ImuErrorState, CopyConstructor)
{
    ImuErrorState x_vec = ImuErrorState::Random();
    ImuErrorState x(x_vec);

    for (int i = 0; i < 9; ++i)
    {
        EXPECT_DOUBLE_EQ(x(i), x_vec(i));
    }
}

}  // namespace models
}  // namespace mc
