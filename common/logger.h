
#pragma once
#include <fstream>
#include <string>
#include <type_traits>

#include <Eigen/Core>

namespace mc {

namespace detail {
template <typename T>
struct isEigen;

template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
struct isEigen<Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>>
{
    using value = std::true_type;
};

template <typename T>
struct isEigen<Eigen::Map<T>>
{
    using value = std::true_type;
};

template <>
struct isEigen<Eigen::MatrixXd>
{
    using value = std::true_type;
};

template <typename T>
struct isEigen
{
    using value = std::false_type;
};
}  // namespace detail

class Logger
{
 public:
    Logger() {}

    Logger(const std::string filename) { open(filename); }

    void open(const std::string& filename) { file_.open(filename); }
    void close() { file_.close(); }

    ~Logger() { file_.close(); }

    template <typename... T>
    void log(T... data)
    {
        int dummy[sizeof...(data)] = {
            (log_impl(data, typename detail::isEigen<decltype(data)>::value()), 1)...};
        (void)dummy;
    }

 private:
    // TODO: Make this multithreaded
    template <typename T>
    void log_impl(T& data, std::true_type const&)
    {
        file_.write((char*)data.data(), sizeof(typename T::Scalar) * data.rows() * data.cols());
    }

    template <typename T>
    void log_impl(T& data, std::false_type const&)
    {
        file_.write((char*)&data, sizeof(T));
    }

    std::ofstream file_;
};
}  // namespace mc
