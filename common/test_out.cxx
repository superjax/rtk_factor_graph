#include <Eigen/Core>

#include "common/error.h"
#include "common/out.h"
#include "gtest/gtest.h"

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

template <typename T>
Error zero_vec3(MatOut<T, 3, 1> v)
{
    (*v) = Eigen::Vector3d::Zero();
    return Error::none();
}

template <typename T, int Rows, int Cols>
Error zero_dyn_mat(MatOut<T, Rows, Cols> v)
{
    v->setZero();
    return Error::none();
}

template <typename T, int Rows>
Error zero_dyn_vec(MatOut<T, Rows, 1> v)
{
    v->setZero();
    return Error::none();
}

TEST(Out, ArrowOperator)
{
    TestStruct test;
    assign_value_arrow(make_out(test));

    EXPECT_DOUBLE_EQ(test.a, 20);
    EXPECT_DOUBLE_EQ(test.b, 30);
    EXPECT_DOUBLE_EQ(test.c, 40);
}

TEST(Out, StarOperator)
{
    TestStruct test;
    assign_value_star(make_out(test));

    EXPECT_DOUBLE_EQ(test.a, 20);
    EXPECT_DOUBLE_EQ(test.b, 30);
    EXPECT_DOUBLE_EQ(test.c, 40);
}

TEST(Out, DoubleCall)
{
    TestStruct test;
    double_assign(make_out(test));

    EXPECT_DOUBLE_EQ(test.a, 20);
    EXPECT_DOUBLE_EQ(test.b, 30);
    EXPECT_DOUBLE_EQ(test.c, 40);
}

TEST(Out, EigenFixedMatrix)
{
    Eigen::Vector3d v = Eigen::Vector3d::Ones();
    zero_vec3(make_out(v));

    EXPECT_EQ(v, Eigen::Vector3d::Zero());
}

TEST(Out, EigenBlock)
{
    Eigen::VectorXd v;
    v.resize(3);
    v.setOnes();
    zero_vec3(make_out(v.head<3>()));

    EXPECT_EQ(v, Eigen::Vector3d::Zero());
}

TEST(Out, EigenDynamicMat)
{
    Eigen::MatrixXd v;
    v.resize(3, 1);
    v.setOnes();
    zero_dyn_mat(make_out(v));

    EXPECT_EQ(v, Eigen::Vector3d::Zero());
}

TEST(Out, EigenDynamicVec)
{
    Eigen::VectorXd v;
    v.resize(3);
    v.setOnes();
    zero_dyn_vec(make_out(v));

    EXPECT_EQ(v, Eigen::Vector3d::Zero());
}

TEST(Out, CallDynamicWithFixed)
{
    Eigen::Vector3d v;
    v.resize(3);
    v.setOnes();
    zero_dyn_vec(make_out(v));

    EXPECT_EQ(v, Eigen::Vector3d::Zero());
}

}  // namespace mc
