#include <gtest/gtest.h>

#include "common/error_result.h"
#include "common/test_helpers.h"

namespace mc {

TEST(Error, OnlyCreateFromConstexprCharArr)
{
    ErrorResult<double> a = Error::create("test");

    EXPECT_FALSE(a.ok());

    EXPECT_THROW(a.res(), std::exception);
    EXPECT_EQ(a.err().what(), "test");
}

TEST(Error, CreateFromNoError)
{
    const auto fn = []() -> ErrorResult<double> { return Error::none(); };
    (void)fn;  // Necessary during release builds, since `EXPECT_DIE` is a stub in Release
    EXPECT_DIE(fn(), "");
}

TEST(Error, CreateErrorResultFromFnReturn)
{
    const auto fn = []() -> ErrorResult<double> { return Error::create("test"); };
    const auto a = fn();

    EXPECT_FALSE(a.ok());
    EXPECT_FALSE(a.err().ok());
    EXPECT_EQ(a.err().what(), "test");
    EXPECT_THROW(a.res(), std::exception);
}

TEST(Error, CreateErrorResultValFromFnReturn)
{
    const auto fn = []() -> ErrorResult<double> { return 3.0; };
    const auto a = fn();

    EXPECT_TRUE(a.ok());
    EXPECT_EQ(a.res(), 3.0);
    EXPECT_THROW(a.err(), std::exception);
}

TEST(Error, ReturnOrAssignValue)
{
    const auto good_fn = []() -> ErrorResult<double> { return 3.0; };

    double val = 0.0;
    double other_val = 0.0;
    const auto scaffold_fn = [&]() -> Error {
        val = RETURN_OR_ASSIGN(good_fn());
        other_val = 100.0;
        return Error::none();
    };
    const Error good_ret = scaffold_fn();

    EXPECT_TRUE(good_ret.ok());
    EXPECT_EQ(val, 3.0);
    EXPECT_EQ(other_val, 100.0);
}

TEST(Error, ReturnOrAssignError)
{
    const auto bad_fn = []() -> ErrorResult<double> { return Error::create("test"); };

    double val = 0.0;
    double other_val = 0.0;
    const auto scaffold_fn = [&]() -> Error {
        val = RETURN_OR_ASSIGN(bad_fn());

        other_val = 100.0;

        return Error::none();
    };
    const Error good_ret = scaffold_fn();

    EXPECT_FALSE(good_ret.ok());
    EXPECT_EQ(good_ret.what(), "test");
    EXPECT_EQ(val, 0.0);
    EXPECT_EQ(other_val, 0.0);
}

}  // namespace mc
