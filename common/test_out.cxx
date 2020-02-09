#include "gtest/gtest.h"

#include "common/error.h"
#include "common/out.h"

namespace mc {

struct TestStruct
{
    double a;
    double b;
    double c;
};

Error assign_value_arrow(Out<TestStruct> test)
{
    test->a = 20;
    test->b = 30;
    test->c = 40;
    return Error::none();
}

Error assign_value_star(Out<TestStruct> test)
{
    (*test).a = 20;
    (*test).b = 30;
    (*test).c = 40;
    return Error::none();
}

Error double_assign(Out<TestStruct> test)
{
    return assign_value_arrow(test);
}

TEST(Out, ArrowOperator)
{
    TestStruct test;
    assign_value_arrow(Out(test));

    EXPECT_DOUBLE_EQ(test.a, 20);
    EXPECT_DOUBLE_EQ(test.b, 30);
    EXPECT_DOUBLE_EQ(test.c, 40);
}

TEST(Out, StarOperator)
{
    TestStruct test;
    assign_value_star(Out(test));

    EXPECT_DOUBLE_EQ(test.a, 20);
    EXPECT_DOUBLE_EQ(test.b, 30);
    EXPECT_DOUBLE_EQ(test.c, 40);
}

TEST(Out, DoubleCall)
{
    TestStruct test;
    double_assign(Out(test));

    EXPECT_DOUBLE_EQ(test.a, 20);
    EXPECT_DOUBLE_EQ(test.b, 30);
    EXPECT_DOUBLE_EQ(test.c, 40);
}

}  // namespace mc
