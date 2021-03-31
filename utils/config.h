#include <Eigen/Core>

#include "common/check.h"
#include "common/error.h"
#include "common/out.h"
#include "utils/split_string.h"
#include "yaml-cpp/node/convert.h"
#include "yaml-cpp/yaml.h"

namespace YAML {
template <typename T, int Rows, int Cols>
struct convert<Eigen::Matrix<T, Rows, Cols>>
{
    static Node encode(const Eigen::Matrix<T, Rows, Cols>& rhs)
    {
        Node node(NodeType::Sequence);
        for (int i = 0; i < Rows; ++i)
        {
            for (int j = 0; j < Cols; ++j)
            {
                node.push_back(rhs(i, j));
            }
        }
        return node;
    }

    static bool decode(const Node& node, Eigen::Matrix<T, Rows, Cols>& rhs)
    {
        if (!node.IsSequence())
            return false;

        rhs.setZero();
        auto it = node.begin();
        for (int i = 0; i < Rows; ++i)
        {
            for (int j = 0; j < Cols; ++j)
            {
                if (it == node.end())
                {
                    return false;
                }
                rhs(i, j) = it->as<T>();
                ++it;
            }
        }
        return true;
    }
};
}  // namespace YAML

namespace mc {
namespace utils {

class Config
{
 public:
    explicit Config(const std::string& config_path);
    explicit Config(const YAML::Node& config);

    template <typename T, int Rows, int Cols>
    bool get(const std::string& key, MatOut<T, Rows, Cols> out, bool required = false)
    {
        std::vector<typename T::Scalar> row_major;
        if (get(key, make_out(row_major), required))
        {
            (*out) = Eigen::Map<Eigen::Matrix<typename T::Scalar, Rows, Cols, Eigen::RowMajor>>(
                row_major.data());
            return true;
        }
        return false;
    }

    template <typename T, int Rows>
    bool get(const std::string& key, MatOut<T, Rows, 1> out, bool required = false)
    {
        std::vector<typename T::Scalar> row_major;
        if (get(key, make_out(row_major), required))
        {
            (*out) = Eigen::Map<Eigen::Matrix<typename T::Scalar, Rows, 1>>(row_major.data());
            return true;
        }
        return false;
    }

    template <typename T>
    bool get(const std::string& key, Out<T> out, bool required = false)
    {
        YAML::Node node = YAML::Clone(yaml_);
        std::vector<std::string> key_path = split_string(key, '/');
        for (size_t i = 0; i < key_path.size(); ++i)
        {
            if (node[key_path[i]])
            {
                if (i == key_path.size() - 1)
                {
                    try
                    {
                        (*out) = node[key_path[i]].as<T>();
                        return true;
                    }
                    catch (const YAML::TypedBadConversion<T>& e)
                    {
                        error("Unable to convert {}", fmt(key));
                        error("{}", fmt(e.what()));
                    }
                }
                else
                {
                    node = node[key_path[i]];
                }
            }
            else
            {
                check(!required, "Missing configuration {}", fmt(key));
                return false;
            }
        }
        return false;
    }

    const YAML::Node yaml_;
};

}  // namespace utils
}  // namespace mc
