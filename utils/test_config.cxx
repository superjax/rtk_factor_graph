
#include <gtest/gtest.h>

#include <cstdio>
#include <experimental/filesystem>
#include <fstream>

#include "common/matrix_defs.h"
#include "common/test_helpers.h"
#include "utils/config.h"

namespace fs = std::experimental::filesystem;

namespace mc {
namespace utils {

class ConfigTest : public ::testing::Test
{
 public:
    void SetUp() override
    {
        temp_file_ = fs::path(testing::TempDir()) / (std::to_string(rand()) + "_config_test.yaml");
        YAML::Node yaml;
        yaml["int"] = 12398;
        yaml["double"] = M_PI;
        yaml["string"] = "String Config";
        for (double i : {1.0, 2.0, 3.0})
        {
            yaml["list"].push_back(i);
        }
        for (double i : {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0})
        {
            yaml["matrix"].push_back(i);
        }
        std::ofstream tmp(temp_file_);
        tmp << YAML::Dump(yaml);
    }

    void TearDown() override
    {
        // Delete the temporary file
        if (fs::exists(temp_file_))
        {
            fs::remove(temp_file_);
        }
    }

    std::string temp_file_;
};

TEST_F(ConfigTest, MakeFromConfig)
{
    EXPECT_NO_THROW(Config cfg(temp_file_));
}

TEST_F(ConfigTest, BadPath)
{
    const std::string bad_path = "not/a/real/file.yaml";

    EXPECT_THROW(Config cfg(bad_path), YAML::BadFile);
}

TEST_F(ConfigTest, ReadValues)
{
    double dbl = 0.0;
    int val = 0;
    std::string str = "";
    std::vector<double> list = {};
    Mat3 mat = Mat3::Zero();
    ;

    Config cfg(temp_file_);

    EXPECT_TRUE(cfg.get("double", make_out(dbl)));
    EXPECT_TRUE(cfg.get("int", make_out(val)));
    EXPECT_TRUE(cfg.get("string", make_out(str)));
    EXPECT_TRUE(cfg.get("list", make_out(list)));
    EXPECT_TRUE(cfg.get("matrix", make_out(mat)));

    EXPECT_EQ(dbl, M_PI);
    EXPECT_EQ(val, 12398);
    EXPECT_EQ(str, "String Config");
    EXPECT_EQ(list.size(), 3ul);
    for (int i : {0, 1, 2})
    {
        EXPECT_EQ(list[i], i + 1);
    }
}

TEST_F(ConfigTest, KeepDefaultNotRequired)
{
    double dbl = 12345.1301;

    Config cfg(temp_file_);

    EXPECT_FALSE(cfg.get("missing", make_out(dbl)));

    EXPECT_EQ(dbl, 12345.1301);
}

TEST_F(ConfigTest, ThrowIfMissingAndRequired)
{
    double dbl = 12345.1301;

    Config cfg(temp_file_);

    EXPECT_DIE(cfg.get("unknown", make_out(dbl), true), "");
    (void)dbl;
}

}  // namespace utils
}  // namespace mc
