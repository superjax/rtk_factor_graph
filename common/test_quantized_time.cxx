#include <gtest/gtest.h>

#include "common/defs.h"
#include "common/quantized_time.h"

namespace mc {

static constexpr double RES = TIME_QUANTIZATION;

TEST(QuantizedTime, GreaterThan)
{
    UTCTime t = UTCTime::now();
    UTCTime tq = UTCTime::now();
    QuantizedTime t2 = (t + 2 * RES).quantized();

    EXPECT_TRUE(t2 >= t);
    EXPECT_TRUE(t2 > t);
    EXPECT_FALSE(t2 == t);
    EXPECT_FALSE(t2 < t);
    EXPECT_FALSE(t2 <= t);

    EXPECT_TRUE(t2 >= tq);
    EXPECT_TRUE(t2 > tq);
    EXPECT_FALSE(t2 == tq);
    EXPECT_FALSE(t2 < tq);
    EXPECT_FALSE(t2 <= tq);

    EXPECT_TRUE(t <= t2);
    EXPECT_TRUE(t < t2);
    EXPECT_FALSE(t == t2);
    EXPECT_FALSE(t > t2);
    EXPECT_FALSE(t >= t2);
}

TEST(QuantizedTime, LessThan)
{
    UTCTime t = UTCTime::now();
    QuantizedTime t2 = (t - 2 * RES).quantized();

    EXPECT_TRUE(t2 <= t);
    EXPECT_TRUE(t2 < t);
    EXPECT_FALSE(t2 == t);
    EXPECT_FALSE(t2 > t);
    EXPECT_FALSE(t2 >= t);

    EXPECT_TRUE(t >= t2);
    EXPECT_TRUE(t > t2);
    EXPECT_FALSE(t == t2);
    EXPECT_FALSE(t < t2);
    EXPECT_FALSE(t <= t2);
}

TEST(QuantizedTime, EqualToBarelyAbove)
{
    UTCTime t = UTCTime::now();
    QuantizedTime t2 = (t + ((RES - 1e-8) / 2.0)).quantized();

    EXPECT_TRUE(t2 <= t);
    EXPECT_FALSE(t2 < t);
    EXPECT_TRUE(t2 == t);
    EXPECT_FALSE(t2 > t);
    EXPECT_TRUE(t2 >= t);

    EXPECT_TRUE(t >= t2);
    EXPECT_FALSE(t > t2);
    EXPECT_TRUE(t == t2);
    EXPECT_FALSE(t < t2);
    EXPECT_TRUE(t <= t2);
}

TEST(QuantizedTime, EqualToBarelyBelow)
{
    UTCTime t = UTCTime::now();
    QuantizedTime t2 = (t - ((RES - 1e-8) / 2.0)).quantized();

    EXPECT_TRUE(t >= t2);
    EXPECT_FALSE(t > t2);
    EXPECT_TRUE(t == t2);
    EXPECT_FALSE(t < t2);
    EXPECT_TRUE(t <= t2);
}

TEST(QuantizedTime, GreaterThanBarelyAbove)
{
    UTCTime t = UTCTime::now();
    QuantizedTime t2 = (t + ((RES + 1e-8) / 2.0)).quantized();

    EXPECT_FALSE(t2 <= t);
    EXPECT_FALSE(t2 < t);
    EXPECT_FALSE(t2 == t);
    EXPECT_TRUE(t2 > t);
    EXPECT_TRUE(t2 >= t);

    EXPECT_FALSE(t >= t2);
    EXPECT_FALSE(t > t2);
    EXPECT_FALSE(t == t2);
    EXPECT_TRUE(t < t2);
    EXPECT_TRUE(t <= t2);
}

TEST(QuantizedTime, LessThanBarelyBelow)
{
    UTCTime t = UTCTime::now();
    QuantizedTime t2 = (t - ((RES + 1e-8) / 2.0)).quantized();

    EXPECT_TRUE(t2 <= t);
    EXPECT_TRUE(t2 < t);
    EXPECT_FALSE(t2 == t);
    EXPECT_FALSE(t2 > t);
    EXPECT_FALSE(t2 >= t);

    EXPECT_TRUE(t >= t2);
    EXPECT_TRUE(t > t2);
    EXPECT_FALSE(t == t2);
    EXPECT_FALSE(t < t2);
    EXPECT_FALSE(t <= t2);
}

}  // namespace mc
