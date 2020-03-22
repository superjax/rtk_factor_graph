#include "common/hetero_collection.h"
#include "core/states/states.h"

#include "gtest/gtest.h"

namespace mc {

struct ItemBase
{
 public:
    int common_member;
};

struct ItemA final : public ItemBase
{
 public:
    double member_a;
};

struct ItemB final : public ItemBase
{
 public:
    uint8_t member_b;
};

struct TestCollectionBase
{
    virtual const std::vector<std::reference_wrapper<const ItemBase>> base_items() const = 0;
};

struct TestCollection : public HeteroCollection<TestCollectionBase, ItemBase, ItemA, ItemA, ItemB>
{
    TestCollection(const ItemA& a1, const ItemA& a2, const ItemB& b)
        : HeteroCollection<TestCollectionBase, ItemBase, ItemA, ItemA, ItemB>(a1, a2, b)
    {
    }
};

TEST(HeteroCollection, Build)
{
    ItemA a1;
    ItemA a2;
    ItemB b;

    TestCollection test_collection(a1, a2, b);
}

TEST(HeteroCollection, Access)
{
    ItemA a1;
    a1.common_member = 1;
    a1.member_a = 10;
    ItemA a2;
    a2.common_member = 2;
    a2.member_a = 20;
    ItemB b;
    b.common_member = 3;
    b.member_b = 30;

    TestCollection test_collection(a1, a2, b);

    EXPECT_EQ(test_collection.get<0>().common_member, 1);
    EXPECT_EQ(test_collection.get<0>().member_a, 10);
    EXPECT_EQ(test_collection.get<1>().common_member, 2);
    EXPECT_EQ(test_collection.get<1>().member_a, 20);
    EXPECT_EQ(test_collection.get<2>().common_member, 3);
    EXPECT_EQ(test_collection.get<2>().member_b, 30);

    EXPECT_EQ(test_collection.base_items()[0].get().common_member, 1);
    EXPECT_EQ(test_collection.base_items()[1].get().common_member, 2);
    EXPECT_EQ(test_collection.base_items()[2].get().common_member, 3);

    EXPECT_EQ(static_cast<const ItemA&>(test_collection.base_items()[0].get()).member_a, 10);
    EXPECT_EQ(static_cast<const ItemA&>(test_collection.base_items()[0].get()).common_member, 1);
    EXPECT_EQ(static_cast<const ItemA&>(test_collection.base_items()[1].get()).member_a, 20);
    EXPECT_EQ(static_cast<const ItemA&>(test_collection.base_items()[1].get()).common_member, 2);
    EXPECT_EQ(static_cast<const ItemB&>(test_collection.base_items()[2].get()).member_b, 30);
    EXPECT_EQ(static_cast<const ItemA&>(test_collection.base_items()[2].get()).common_member, 3);
}

TEST(HeteroCollection, Modify)
{
    ItemA a1;
    a1.common_member = 1;
    a1.member_a = 10;
    ItemA a2;
    a2.common_member = 2;
    a2.member_a = 20;
    ItemB b;
    b.common_member = 3;
    b.member_b = 30;

    TestCollection test_collection(a1, a2, b);

    EXPECT_EQ(test_collection.get<0>().common_member, 1);
    EXPECT_EQ(test_collection.get<0>().member_a, 10);

    a1.member_a += 11;

    EXPECT_EQ(test_collection.get<0>().member_a, 21);
}

}  // namespace mc
