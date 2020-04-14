
#pragma once
#include <Eigen/Core>
#include <experimental/filesystem>
#include <fstream>
#include <string>
#include <type_traits>

#include "math/dquat.h"
#include "utils/file.h"

namespace mc {

namespace detail {

template <typename T>
struct has_data
{
 private:
    template <typename U>
    static auto test(int) -> decltype(std::declval<U>().data(), std::true_type());

    template <typename>
    static std::false_type test(...);

 public:
    static constexpr bool value = std::is_same<decltype(test<T>(0)), std::true_type>::value;
};

}  // namespace detail

class Logger
{
 public:
    Logger() {}

    Logger(const std::string filename) { open(filename); }

    void open(const std::string& filename)
    {
        namespace fs = std::experimental::filesystem;
        fs::path directory = fs::path(filename).parent_path();
        utils::makeDirectoryIfNotExist(directory);
        file_.open(filename);
    }
    void close() { file_.close(); }

    ~Logger() { file_.close(); }

    template <typename... T>
    void log(T... data)
    {
        int dummy[sizeof...(data)] = {(log_impl(data), 1)...};
        (void)dummy;
    }

 private:
    // TODO: Make this multithreaded
    template <typename T>
    void log_impl(T& data)
    {
        if constexpr (detail::has_data<T>::value)
        {
            file_.write((char*)data.data(), sizeof(typename T::Scalar) * data.rows() * data.cols());
        }
        else
        {
            file_.write((char*)&data, sizeof(T));
        }
    }

    std::ofstream file_;
};
}  // namespace mc
