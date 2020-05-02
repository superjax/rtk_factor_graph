#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <sstream>
#include <string>
#include <type_traits>
#include <typeinfo>

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/out.h"
#include "common/error.h"
#include "common/utctime.h"

namespace mc {

namespace logging {
namespace detail {

enum
{
    FP64,
    FP32,
    INT64,
    INT32,
    INT16,
    INT8,
    UINT64,
    UINT32,
    UINT16,
    UINT8,
};
// clang-format off
template <typename type>
struct get_type;
template <> struct get_type<double> { static constexpr int value = FP64; };
template <> struct get_type<float>  { static constexpr int value = FP32; };
template <> struct get_type<int64_t>  { static constexpr int value = INT64; };
template <> struct get_type<int32_t>  { static constexpr int value = INT32; };
template <> struct get_type<int16_t>  { static constexpr int value = INT16; };
template <> struct get_type<int8_t>  { static constexpr int value = INT8; };
template <> struct get_type<uint64_t>  { static constexpr int value = UINT64; };
template <> struct get_type<uint32_t>  { static constexpr int value = UINT32; };
template <> struct get_type<uint16_t>  { static constexpr int value = UINT16; };
template <> struct get_type<uint8_t>  { static constexpr int value = UINT8; };
// clang-format on

// These functions are never defined.
template <typename T>
std::true_type is_eigen_test(const Eigen::MatrixBase<T>*);
std::false_type is_eigen_test(...);

template <typename T>
struct is_eigen : public decltype(is_eigen_test(std::declval<T*>()))
{
};

}  // namespace detail

template <typename T, std::enable_if_t<detail::is_eigen<T>::value, int> = 0>
std::string format(const T& val, const std::string& name, int pad = 0)
{
    std::stringstream ss;
    ss << std::string(pad, ' ') << name << ": {\n";
    ss << std::string(pad + 2, ' ') << "type: " << detail::get_type<typename T::Scalar>::value
       << ",\n";
    ss << std::string(pad + 2, ' ') << "rows: " << val.rows() << ",\n";
    ss << std::string(pad + 2, ' ') << "cols: " << val.cols() << "\n";
    ss << std::string(pad, ' ') << "}";
    return ss.str();
}

template <typename T, std::enable_if_t<!detail::is_eigen<T>::value, int> = 0>
std::string format(const T& val, const std::string& name, int pad = 0)
{
    (void)val;
    std::stringstream ss;
    ss << std::string(pad, ' ') << name << ": {\n";
    ss << std::string(pad + 2, ' ') << "type: " << detail::get_type<T>::value << "\n";
    ss << std::string(pad, ' ') << "}";
    return ss.str();
}

template <typename T>
std::string format(const T* val, const std::string& name, int size, int pad = 0)
{
    (void)val;
    std::stringstream ss;
    ss << std::string(pad, ' ') << name << ": {\n";
    ss << std::string(pad + 2, ' ') << "type: " << detail::get_type<T>::value << ",\n";
    ss << std::string(pad + 2, ' ') << "rows: " << size << ",\n";
    ss << std::string(pad, ' ') << "}";
    return ss.str();
}

std::string format(const UTCTime& eph, const std::string& name, int pad = 0);
std::string format(const ephemeris::GPSEphemeris& eph, const std::string& name, int pad = 0);
std::string format(const ephemeris::GalileoEphemeris& eph, const std::string& name, int pad = 0);
std::string format(const ephemeris::GlonassEphemeris& eph, const std::string& name, int pad = 0);

template <typename... Args, size_t... I>
std::string __make_header_impl(const std::array<std::string, sizeof...(Args)>& names,
                               std::index_sequence<I...>,
                               const Args&... args)
{
    std::stringstream ss;
    const auto fun = [&](const std::string& str) {
        ss << str << ",\n";
        return 1;
    };
    int dummy[] = {fun(format(args, names[I], 2))...};
    (void)dummy;
    return ss.str();
}

template <typename... Args>
std::string makeHeader(const std::array<std::string, sizeof...(Args)>& names, const Args&... args)
{
    return __make_header_impl(names, std::index_sequence_for<Args...>{}, args...);
}

Error getHeaderTime(const std::string& header_file, Out<UTCTime> t0);

};  // namespace logging
}  // namespace mc
