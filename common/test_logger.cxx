#include <gtest/gtest.h>
#include <Eigen/Core>

#include "common/logger.h"
#include "common/test_helpers.h"

namespace mc {

TEST(Logger, LogScalars)
{
    double a = 42.0;
    float b = 132.0;
    int c = 15;
    int64_t d = 87;
    {
        Logger log("/tmp/logger_test.bin");
        log.log(a, b, c, d);
    }

    std::ifstream check("/tmp/logger_test.bin");
    struct Test
    {
        double a;
        float b;
        int c;
        int64_t d;
    };
    Test test;
    check.read((char*)&test, sizeof(test));

    EXPECT_EQ(a, test.a);
    EXPECT_EQ(b, test.b);
    EXPECT_EQ(c, test.c);
    EXPECT_EQ(d, test.d);
}

TEST(Logger, LogMatrices)
{
    using namespace Eigen;
    Matrix<double, 3, 4> a = Matrix<double, 3, 4>::Random();
    float b = 132.0;
    Matrix<int, 7, 2> c = Matrix<int, 7, 2>::Random();
    Matrix<float, 12, 1> d = Matrix<float, 12, 1>::Random();
    Matrix<double, 1, 4> e = Matrix<double, 1, 4>::Random();
    int f = 15;
    int64_t g = 87;
    {
        Logger log("/tmp/logger_test.bin");
        log.log(a, b, c, d, e, f, g);
    }

    std::ifstream check("/tmp/logger_test.bin");
    struct Test
    {
        double a[12];
        float b;
        int c[7 * 2];
        float d[12];
        double e[4];
        int f;
        int64_t g;
    } __attribute__((packed));
    Test test;
    check.read((char*)&test, sizeof(test));

    Map<Matrix<double, 3, 4>> test_a(test.a);
    Map<Matrix<int, 7, 2>> test_c(test.c);
    Map<Matrix<float, 12, 1>> test_d(test.d);
    Map<Matrix<double, 1, 4>> test_e(test.e);
    MATRIX_EQUALS(a, test_a);
    EXPECT_EQ(b, test.b);
    MATRIX_EQUALS(c, test_c);
    MATRIX_EQUALS(d, test_d);
    MATRIX_EQUALS(e, test_e);
    EXPECT_EQ(f, test.f);
    EXPECT_EQ(g, test.g);
}

TEST(Logger, LogKindsOfMatrices)
{
    using namespace Eigen;
    Matrix<double, 3, 4> a = Matrix<double, 3, 4>::Random();

    MatrixXd b;
    b.resize(3, 5);
    b.setRandom();

    float buf[25];
    Map<Matrix<float, 5, 5>> c(buf);
    c.setRandom();

    {
        Logger log("/tmp/logger_test.bin");
        log.log(a, b, c);
    }

    std::ifstream check("/tmp/logger_test.bin");
    struct Test
    {
        double a[12];
        double b[15];
        float c[25];
    } __attribute__((packed));
    Test test;
    check.read((char*)&test, sizeof(test));

    Map<Matrix<double, 3, 4>> test_a(test.a);
    Map<Matrix<double, 3, 5>> test_b(test.b);
    Map<Matrix<float, 5, 5>> test_c(test.c);
    MATRIX_EQUALS(a, test_a);
    MATRIX_EQUALS(b, test_b);
    MATRIX_EQUALS(c, test_c);
}

}  // namespace mc
