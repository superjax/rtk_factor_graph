#include <gtest/gtest.h>

#include <Eigen/Core>

#include "common/logging/header.h"
#include "common/logging/log_reader.h"
#include "common/logging/logger.h"
#include "common/test_helpers.h"

namespace mc {
namespace logging {

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
    struct TestObj
    {
        double a;
        float b;
        int c;
        int64_t d;
    } __attribute((__packed__));
    TestObj test;
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
    struct TestObj
    {
        double a[12];
        float b;
        int c[7 * 2];
        float d[12];
        double e[4];
        int f;
        int64_t g;
    } __attribute__((packed));
    TestObj test;
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
    struct TestObj
    {
        double a[12];
        double b[15];
        float c[25];
    } __attribute__((packed));
    TestObj test;
    check.read((char*)&test, sizeof(test));

    Map<Matrix<double, 3, 4>> test_a(test.a);
    Map<Matrix<double, 3, 5>> test_b(test.b);
    Map<Matrix<float, 5, 5>> test_c(test.c);
    MATRIX_EQUALS(a, test_a);
    MATRIX_EQUALS(b, test_b);
    MATRIX_EQUALS(c, test_c);
}

TEST(Logger, WithHeader)
{
    Logger logger("/tmp/test_header");
    logger.addHeader(
        logging::makeHeader({"a", "b", "c", "e", "f"}, Vec3f(), Vec4(), Mat96(), 1, 2.0));
}

TEST(Logger, CustomTypes)
{
    struct A
    {
        ephemeris::GPSEphemeris gps_eph{0};
        ephemeris::GlonassEphemeris glo_eph{0};
        ephemeris::GalileoEphemeris gal_eph{0};
        UTCTime t;
    };

    A test;
    test.gal_eph.sat = 102;
    test.gal_eph.toe.sec = rand();
    test.gal_eph.toe.nsec = rand();
    test.glo_eph.toe.sec = rand();
    test.glo_eph.toe.nsec = rand();
    test.gps_eph.toe.sec = rand();
    test.gps_eph.toe.nsec = rand();
    test.t.sec = rand();
    test.t.nsec = rand();
    test.gps_eph.health = 4;
    test.gal_eph.e1b_dvs = 51;
    test.glo_eph.pos.setRandom();
    test.glo_eph.vel.setRandom();
    test.glo_eph.acc.setRandom();

    {
        Logger logger("/tmp/custom_types.log");
        logger.log(test.gps_eph, test.glo_eph, test.gal_eph, test.t);
    }

    A loaded = {};

    {
        LogReader log_reader("/tmp/custom_types.log");
        log_reader.read(loaded.gps_eph, loaded.glo_eph, loaded.gal_eph, loaded.t);
    }

    EXPECT_EQ(loaded.gal_eph.sat, test.gal_eph.sat);
    EXPECT_EQ(loaded.gal_eph.toe.sec, test.gal_eph.toe.sec);
    EXPECT_EQ(loaded.gal_eph.toe.nsec, test.gal_eph.toe.nsec);
    EXPECT_EQ(loaded.glo_eph.toe.sec, test.glo_eph.toe.sec);
    EXPECT_EQ(loaded.glo_eph.toe.nsec, test.glo_eph.toe.nsec);
    EXPECT_EQ(loaded.gps_eph.toe.sec, test.gps_eph.toe.sec);
    EXPECT_EQ(loaded.gps_eph.toe.nsec, test.gps_eph.toe.nsec);
    EXPECT_EQ(loaded.t.sec, test.t.sec);
    EXPECT_EQ(loaded.t.nsec, test.t.nsec);
    EXPECT_EQ(loaded.gps_eph.health, test.gps_eph.health);
    EXPECT_EQ(loaded.gal_eph.e1b_dvs, test.gal_eph.e1b_dvs);
    MATRIX_EQUALS(loaded.glo_eph.pos, test.glo_eph.pos);
    MATRIX_EQUALS(loaded.glo_eph.vel, test.glo_eph.vel);
    MATRIX_EQUALS(loaded.glo_eph.acc, test.glo_eph.acc);
}

}  // namespace logging
}  // namespace mc
