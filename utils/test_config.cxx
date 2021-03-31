
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
        yaml["nested"]["string"] = "Nested String Config";
        yaml["nested"]["int"] = 456;
        yaml["nested"]["double"] = 456.392;

        for (double i : {1.0, 2.0, 3.0})
        {
            yaml["list"].push_back(i);
            yaml["nested"]["list"].push_back(i + 3);
        }
        for (double i : {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0})
        {
            yaml["matrix"].push_back(i);
            yaml["nested"]["matrix"].push_back(i + 9.0);
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
    double nested_dbl = 0.0;
    int val = 0;
    int nested_val = 0;
    std::string str = "";
    std::string nested_str = "";
    std::vector<double> list = {};
    std::vector<double> nested_list = {};
    Mat3 mat = Mat3::Zero();
    Mat3 nested_mat = Mat3::Zero();

    Config cfg(temp_file_);

    EXPECT_TRUE(cfg.get("double", make_out(dbl)));
    EXPECT_TRUE(cfg.get("int", make_out(val)));
    EXPECT_TRUE(cfg.get("string", make_out(str)));
    EXPECT_TRUE(cfg.get("list", make_out(list)));
    EXPECT_TRUE(cfg.get("matrix", make_out(mat)));

    EXPECT_TRUE(cfg.get("nested/int", make_out(nested_val)));
    EXPECT_TRUE(cfg.get("nested/double", make_out(nested_dbl)));
    EXPECT_TRUE(cfg.get("nested/string", make_out(nested_str)));
    EXPECT_TRUE(cfg.get("nested/list", make_out(nested_list)));
    EXPECT_TRUE(cfg.get("nested/matrix", make_out(nested_mat)));

    EXPECT_EQ(dbl, M_PI);
    EXPECT_EQ(nested_dbl, 456.392);
    EXPECT_EQ(val, 12398);
    EXPECT_EQ(nested_val, 456);
    EXPECT_EQ(str, "String Config");
    EXPECT_EQ(nested_str, "Nested String Config");
    EXPECT_EQ(list.size(), 3ul);
    EXPECT_EQ(nested_list.size(), 3ul);
    for (int i : {0, 1, 2})
    {
        EXPECT_EQ(list[i], i + 1);
        EXPECT_EQ(nested_list[i], i + 4);
    }
    EXPECT_EQ(mat, (Mat3() << 1, 2, 3, 4, 5, 6, 7, 8, 9).finished());
    EXPECT_EQ(nested_mat, (Mat3() << 10, 11, 12, 13, 14, 15, 16, 17, 18).finished());
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
