#include "common/logging/log_writer.h"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "common/test_helpers.h"

namespace mc {
namespace logging {

namespace fs = std::experimental::filesystem;

static const std::string log_directory = "/tmp/logger_test";

TEST(Logger, LogScalars)
{
    constexpr int LOG_KEY = 12934;
    double a = 42.0;
    float b = 132.0;
    int c = 15;
    int64_t d = 87;
    std::string log_path;
    std::string log_id;
    {
        Logger log(log_directory);
        log.initStream<double, float, int, int64_t>(LOG_KEY, {"a", "b", "c", "d"});
        log.logStamped(LOG_KEY, a, b, c, d);
        log_path = log_directory + "/" + log.logId();
        log_id = log.logId();
    }

    const std::string data_path = log_path + fmt::format("/LOG_KEY_{}.log", LOG_KEY);
    const std::string data_format_path = log_path + fmt::format("/LOG_KEY_{}.yml", LOG_KEY);
    const std::string manifest_path = log_path + "/manifest.yml";

    // Ensure that all  the files are there
    EXPECT_TRUE(fs::exists(log_path));
    EXPECT_TRUE(fs::exists(data_path));
    EXPECT_TRUE(fs::exists(data_format_path));
    EXPECT_TRUE(fs::exists(manifest_path));

    std::ifstream check(data_path);
    struct TestObj
    {
        UTCTime time;
        double a;
        float b;
        int c;
        int64_t d;
    };
    TestObj test;
    check.read((char*)&test, sizeof(test));

    EXPECT_EQ(a, test.a);
    EXPECT_EQ(b, test.b);
    EXPECT_EQ(c, test.c);
    EXPECT_EQ(d, test.d);

    const auto manifest = YAML::LoadFile(manifest_path);
    EXPECT_EQ(manifest["id"].as<std::string>(), log_id);
    EXPECT_EQ(manifest["keys"].size(), 1ul);
    EXPECT_EQ(manifest["keys"][0].as<int>(), LOG_KEY);

    const auto format = YAML::LoadFile(data_format_path);
    EXPECT_EQ(format["key"].as<int>(), LOG_KEY);
    EXPECT_EQ(format["num_records"].as<int>(), 1);
    EXPECT_EQ(format["format"]["a"]["type"].as<int>(), detail::get_type<double>::value);
    EXPECT_EQ(format["format"]["b"]["type"].as<int>(), detail::get_type<float>::value);
    EXPECT_EQ(format["format"]["c"]["type"].as<int>(), detail::get_type<int>::value);
    EXPECT_EQ(format["format"]["d"]["type"].as<int>(), detail::get_type<int64_t>::value);
}

TEST(Logger, LogMatrices)
{
    constexpr int LOG_KEY = 1235312;
    using namespace Eigen;
    Matrix<double, 3, 4> a = Matrix<double, 3, 4>::Random();
    float b = 132.0;
    Matrix<int, 7, 2> c = Matrix<int, 7, 2>::Random();
    Matrix<float, 12, 1> d = Matrix<float, 12, 1>::Random();
    Matrix<double, 1, 4> e = Matrix<double, 1, 4>::Random();
    int f = 15;
    int64_t g = 87;
    std::string log_path;
    std::string log_id;
    {
        Logger log(log_directory);

        log.initStream<decltype(a), decltype(b), decltype(c), decltype(d), decltype(e), decltype(f),
                       decltype(g)>(LOG_KEY, {"a", "b", "c", "d", "e", "f", "g"});
        log.logStamped(LOG_KEY, a, b, c, d, e, f, g);
        log_path = log_directory + "/" + log.logId();
        log_id = log.logId();
    }

    const std::string data_path = log_path + fmt::format("/LOG_KEY_{}.log", LOG_KEY);
    std::ifstream check(data_path);
    struct TestObj
    {
        UTCTime time;
        double a[3 * 4];
        float b;
        int c[7 * 2];
        float d[12];
        double e[4];
        int f;
        int64_t g;
    };

    TestObj test;
    check.read((char*)&test.time, sizeof(test.time));
    check.read((char*)&test.a, sizeof(test.a));
    check.read((char*)&test.b, sizeof(test.b));
    check.read((char*)&test.c, sizeof(test.c));
    check.read((char*)&test.d, sizeof(test.d));
    check.read((char*)&test.e, sizeof(test.e));
    check.read((char*)&test.f, sizeof(test.f));
    check.read((char*)&test.g, sizeof(test.g));

    Map<Matrix<double, 3, 4>> test_a(test.a);
    Map<Matrix<int, 7, 2>> test_c(test.c);
    Map<Matrix<float, 12, 1>> test_d(test.d);
    Map<Matrix<double, 1, 4>> test_e(test.e);

    MAT_EQ(a, test_a);
    EXPECT_EQ(b, test.b);
    MAT_EQ(c, test_c);
    MAT_EQ(d, test_d);
    MAT_EQ(e, test_e);
    EXPECT_EQ(f, test.f);
    EXPECT_EQ(g, test.g);
}

TEST(Logger, LogKindsOfMatrices)
{
    using namespace Eigen;

    constexpr int LOG_KEY = 351234;
    Matrix<double, 3, 4> a = Matrix<double, 3, 4>::Random();

    MatrixXd b;
    b.resize(3, 5);
    b.setRandom();

    float buf[25];
    Map<Matrix<float, 5, 5>> c(buf);
    c.setRandom();
    std::string log_path;
    std::string log_id;
    {
        Logger log(log_directory);
        log.initStream<Matrix<double, 3, 4>, Matrix<double, 3, 5>, Matrix<float, 5, 5>>(
            LOG_KEY, {"a", "b", "c"});
        log.logStamped(LOG_KEY, a, b, c);
        log_path = log_directory + "/" + log.logId();
        log_id = log.logId();
    }

    const std::string data_path = log_path + fmt::format("/LOG_KEY_{}.log", LOG_KEY);

    std::ifstream check(data_path);
    struct TestObj
    {
        UTCTime time;
        double a[12];
        double b[15];
        float c[25];
    };
    TestObj test;
    check.read((char*)&test, sizeof(test));

    Map<Matrix<double, 3, 4>> test_a(test.a);
    Map<Matrix<double, 3, 5>> test_b(test.b);
    Map<Matrix<float, 5, 5>> test_c(test.c);
    MAT_EQ(a, test_a);
    MAT_EQ(b, test_b);
    MAT_EQ(c, test_c);
}

// TEST(Logger, CustomTypes)
// {
//     using namespace ephemeris;
//     struct A
//     {
//         GPSEphemeris gps_eph{0};
//         GlonassEphemeris glo_eph{0};
//         GalileoEphemeris gal_eph{0};
//         UTCTime t;
//     };

//     constexpr int LOG_KEY = 351234;
//     std::string log_path;
//     std::string log_id;

//     A test;
//     test.gal_eph.sat = 102;
//     test.gal_eph.toe.sec = rand();
//     test.gal_eph.toe.nsec = rand();
//     test.glo_eph.toe.sec = rand();
//     test.glo_eph.toe.nsec = rand();
//     test.gps_eph.toe.sec = rand();
//     test.gps_eph.toe.nsec = rand();
//     test.t.sec = rand();
//     test.t.nsec = rand();
//     test.gps_eph.health = 4;
//     test.gal_eph.e1b_dvs = 51;
//     test.glo_eph.pos.setRandom();
//     test.glo_eph.vel.setRandom();
//     test.glo_eph.acc.setRandom();

//     {
//         Logger logger(log_directory);
//         logger.initStream<GPSEphemeris, GlonassEphemeris, GalileoEphemeris, UTCTime>(
//             LOG_KEY, "gps", "glo", "gal", "t");
//         logger.log(LOG_KEY, test.gps_eph, test.glo_eph, test.gal_eph, test.t);

//         log_path = log_directory + "/" + log.logId();
//     }
//     const std::string data_path = log_path + fmt::format("/LOG_KEY_{}.log", LOG_KEY);

//     A loaded = {};

//     {
//         LogReader log_reader(data_path);
//         log_reader.read(loaded.gps_eph, loaded.glo_eph, loaded.gal_eph, loaded.t);
//     }

//     EXPECT_EQ(loaded.gal_eph.sat, test.gal_eph.sat);
//     EXPECT_EQ(loaded.gal_eph.toe.sec, test.gal_eph.toe.sec);
//     EXPECT_EQ(loaded.gal_eph.toe.nsec, test.gal_eph.toe.nsec);
//     EXPECT_EQ(loaded.glo_eph.toe.sec, test.glo_eph.toe.sec);
//     EXPECT_EQ(loaded.glo_eph.toe.nsec, test.glo_eph.toe.nsec);
//     EXPECT_EQ(loaded.gps_eph.toe.sec, test.gps_eph.toe.sec);
//     EXPECT_EQ(loaded.gps_eph.toe.nsec, test.gps_eph.toe.nsec);
//     EXPECT_EQ(loaded.t.sec, test.t.sec);
//     EXPECT_EQ(loaded.t.nsec, test.t.nsec);
//     EXPECT_EQ(loaded.gps_eph.health, test.gps_eph.health);
//     EXPECT_EQ(loaded.gal_eph.e1b_dvs, test.gal_eph.e1b_dvs);
//     MAT_EQ(loaded.glo_eph.pos, test.glo_eph.pos);
//     MAT_EQ(loaded.glo_eph.vel, test.glo_eph.vel);
//     MAT_EQ(loaded.glo_eph.acc, test.glo_eph.acc);
// }

}  // namespace logging
}  // namespace mc
