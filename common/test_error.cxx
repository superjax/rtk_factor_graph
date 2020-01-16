#include <gtest/gtest.h>

#include "common/error.h"

TEST(Error, OnlyCreateFromConstexprCharArr)
{
    Error a = Error::create("test");

    EXPECT_FALSE(a.ok());

    // This should not compile
    // std::string not_constant_message = "message";
    // Error b = Error::create(not_constant_message.c_str());
}

TEST(Error, NoError)
{
    Error none = Error::none();
    EXPECT_TRUE(none.ok());
}

TEST(Error, EqOperator)
{
    static constexpr char message[] = "Test 1";
    Error a = Error::create(message);
    Error b = Error::create(message);
    EXPECT_EQ(a, b);

    Error c = Error::create("Test 1");  // This error has the same text, but not the same address
    EXPECT_NE(a, c);

    EXPECT_EQ(a, message);
    EXPECT_NE(a, "Test 1");

    EXPECT_NE(c, message);

    static constexpr char message2[] = "Test 2";
    Error d = Error::create(message2);
    EXPECT_NE(a, d);
}

TEST(Error, Stream)
{
    static constexpr char message[] = "Test 1";

    std::ostringstream out;
    Error a = Error::create(message);
    out << a;
    EXPECT_EQ(message, out.str());

    EXPECT_STREQ(a.what(), "Test 1");
    EXPECT_EQ(a.what(), message);
}
