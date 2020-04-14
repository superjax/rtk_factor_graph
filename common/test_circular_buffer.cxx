#include <gtest/gtest.h>

#include "common/circular_buffer.h"
#include "common/print.h"
#include "utils/non_copyable.h"

namespace mc {

TEST(CircularBuffer, EmptyFull)
{
    CircularBuffer<double> buf;
    EXPECT_TRUE(buf.empty());
    EXPECT_FALSE(buf.full());
    for (int i = 0; i < buf.SIZE - 1; ++i)
    {
        buf.emplace_back(i);
        EXPECT_FALSE(buf.empty());
        if (i == buf.SIZE - 2)
        {
            EXPECT_TRUE(buf.full());
        }
        else
        {
            EXPECT_FALSE(buf.full());
        }
    }
    EXPECT_FALSE(buf.empty());
}

void exercise(const CircularBuffer<double>& buf)
{
    // iterate over whole vector forwards
    size_t i = 0;
    for (auto it = buf.begin(); it != buf.end(); ++it)
    {
        EXPECT_EQ(*it, i);
        EXPECT_EQ(it.idx(), i);
        EXPECT_EQ(buf[i], i);
        EXPECT_EQ(buf.at(i), i);
        i++;
    }
    EXPECT_EQ(i, buf.size());
    EXPECT_EQ(i, buf.capacity());
    EXPECT_EQ(i, buf.max_size());

    // iterate forwards by non-unit increments
    auto it = buf.begin() + 10;
    for (i = 10; i < buf.back(); i += 3)
    {
        EXPECT_EQ(*it, i);
        it += 3;
    }

    // iterate over whole vector backwards
    i = buf.back();
    for (it = buf.end() - 1; it != buf.begin() - 1; --it)
    {
        EXPECT_EQ(*it, i);
        EXPECT_EQ(it.idx(), i);
        EXPECT_EQ(buf[i], i);
        EXPECT_EQ(buf.at(i), i);
        i--;
    }

    // iterate backwards by non-unit increments
    it = buf.end() - 11;
    for (int j = buf.back() - 10; j > buf.front(); j -= 3)
    {
        EXPECT_EQ(*it, static_cast<size_t>(j));
        it -= 3;
    }

    // Subtract two iterators
    auto it1 = buf.begin() + 25;
    auto it2 = buf.end() - 33;
    EXPECT_EQ(it2 - it1, static_cast<int>(buf.size() - 33 - 25));
    EXPECT_EQ(it1 - it2, -static_cast<int>(buf.size() - 33 - 25));
}

TEST(CircularBuffer, IteratorOpsFullVector)
{
    CircularBuffer<double> buf;
    for (int i = 0; i < buf.SIZE - 1; ++i)
    {
        buf.emplace_back(i);
    }
}

TEST(CircularBuffer, IteratorOpsHalfVector)
{
    CircularBuffer<double> buf;
    for (int i = 0; i < buf.SIZE / 2; ++i)
    {
        buf.emplace_back(i);
    }
}

TEST(CircularBuffer, IteratorOpsBackHalfVector)
{
    CircularBuffer<double> buf;
    for (int i = 0; i < buf.SIZE - 1; ++i)
    {
        buf.emplace_back(i - buf.SIZE / 2);
    }
    for (int i = 0; i < buf.SIZE / 2; ++i)
    {
        buf.pop_front();
    }
}

TEST(CircularBuffer, IteratorOpsWrapHalf)
{
    CircularBuffer<double> buf;
    for (int i = 0; i < 2 * buf.SIZE / 3; ++i)
    {
        buf.emplace_back(i - buf.SIZE / 2);
    }
    for (int i = 0; i < 2 * buf.SIZE / 3; ++i)
    {
        buf.pop_front();
    }
    EXPECT_TRUE(buf.empty());

    for (int i = 0; i < buf.SIZE / 2; ++i)
    {
        buf.emplace_back(i);
    }
}

TEST(CircularBuffer, emplace)
{
    CircularBuffer<double> buf;
    buf.emplace_back(3.0);
    buf.emplace_back(6.0);
    buf.emplace_back(10.0);
    buf.emplace_front(11.0);

    EXPECT_EQ(buf.data()[0], 3.0);
    EXPECT_EQ(buf.data()[1], 6.0);
    EXPECT_EQ(buf.data()[2], 10.0);
    EXPECT_EQ(buf.data()[buf.SIZE - 1], 11.0);

    EXPECT_EQ(buf.begin().idx(), 0u);
    EXPECT_EQ(buf.front(), 11.0);
    EXPECT_EQ(buf.end().idx(), 4u);  // should be one after the last element we placed
    EXPECT_EQ(buf.back(), 10.0);
}

static int destructor_calls = 0;
static int constructor_calls = 0;
struct TestType : public utils::NonCopyable
{
    TestType(double _a, double _b, double _c) : a(_a), b(_b), c(_c) { ++constructor_calls; }
    ~TestType() { ++destructor_calls; }
    double a;
    double b;
    int c;
};

TEST(CircularBuffer, NonCopyableType)
{
    CircularBuffer<TestType> buf;
    destructor_calls = 0;
    constructor_calls = 0;

    // buf.push_back({0.0, 0.0, 0}); // This should not compile for a non-copyable type
    buf.emplace_back(1.0, 2.0, 3);
    buf.emplace_back(2.0, 3.0, 4);
    buf.emplace_front(3.0, 4.0, 5);
    buf.emplace_front(4.0, 5.0, 6);

    EXPECT_FALSE(buf.empty());
    EXPECT_EQ(buf.size(), 4u);
    EXPECT_EQ(constructor_calls, 4);

    buf.pop_back();

    EXPECT_FALSE(buf.empty());
    EXPECT_EQ(buf.size(), 3u);
    EXPECT_EQ(buf.back().a, 1.0);
    EXPECT_EQ(buf.front().a, 4.0);

    buf.pop_front();

    EXPECT_FALSE(buf.empty());
    EXPECT_EQ(buf.size(), 2u);
    EXPECT_EQ(buf.back().a, 1.0);
    EXPECT_EQ(buf.front().a, 3.0);

    buf.pop_front();
    EXPECT_FALSE(buf.empty());
    EXPECT_EQ(buf.size(), 1u);
    EXPECT_EQ(buf.back().a, 1.0);
    EXPECT_EQ(buf.front().a, 1.0);
    EXPECT_EQ(buf.begin(), buf.end() - 1);

    buf.pop_front();
    EXPECT_TRUE(buf.empty());
    EXPECT_EQ(buf.size(), 0u);

    EXPECT_EQ(destructor_calls, constructor_calls);
}

TEST(CircularBuffer, CallDestructorsWhenDestroyed)
{
    destructor_calls = 0;
    constructor_calls = 0;
    {
        CircularBuffer<TestType> buf;

        // buf.push_back({0.0, 0.0, 0}); // This should not compile for a non-copyable type
        buf.emplace_back(1.0, 2.0, 3);
        buf.emplace_back(2.0, 3.0, 4);
        buf.emplace_front(3.0, 4.0, 5);
        buf.emplace_front(4.0, 5.0, 6);
    }

    EXPECT_EQ(constructor_calls, 4);
    EXPECT_EQ(destructor_calls, 4);
}

TEST(CircularBuffer, CallDestructorsWhenCleared)
{
    destructor_calls = 0;
    constructor_calls = 0;
    CircularBuffer<TestType> buf;

    // buf.push_back({0.0, 0.0, 0}); // This should not compile for a non-copyable type
    buf.emplace_back(1.0, 2.0, 3);
    buf.emplace_back(2.0, 3.0, 4);
    buf.emplace_front(3.0, 4.0, 5);
    buf.emplace_front(4.0, 5.0, 6);

    buf.clear();

    EXPECT_TRUE(buf.empty());
    EXPECT_EQ(constructor_calls, 4);
    EXPECT_EQ(destructor_calls, 4);
}

TEST(CircularBuffer, ModifyElements)
{
    destructor_calls = 0;
    constructor_calls = 0;
    CircularBuffer<TestType> buf;

    for (int i = 0; i < buf.SIZE / 2; ++i)
    {
        buf.emplace_back(i, i, i);
    }

    for (int i = 0; i < buf.SIZE / 2; ++i)
    {
        buf[i].a *= 2.0;
    }

    for (auto it = buf.begin(); it != buf.end(); it++)
    {
        it->b *= 3.0;
    }
}

TEST(CircularBuffer, DontModifyElementsOfConst)
{
    const auto fun = [](const CircularBuffer<TestType>& buf) {
        // None of these should compile
        // buf[0].a = 1.0;
        // buf.begin()->a *= 3.0;
        // buf.end()->a *= 3.0;
    };

    CircularBuffer<TestType> buf;
    buf.emplace_back(1, 2, 3);

    fun(buf);
}

}  // namespace mc
