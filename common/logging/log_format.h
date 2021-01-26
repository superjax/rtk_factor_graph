#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <sstream>
#include <string>
#include <type_traits>
#include <typeinfo>

#include "common/error.h"
#include "common/out.h"
#include "common/print.h"

namespace mc {

// Forward declaration for special serialize types
class UTCTime;
namespace ephemeris {
class GPSEphemeris;
class GlonassEphemeris;
class GalileoEphemeris;
}  // namespace ephemeris
namespace meas {
class ImuSample;
class GnssObservation;
}  // namespace meas
namespace math {
template <typename T>
class Quat;
template <typename T>
class TwoJet;
template <typename T>
class DQuat;
template <typename T>
class Jet;
}  // namespace math

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
YAML::Node format(const T& val)
{
    YAML::Node node;
    node["type"] = detail::get_type<typename T::Scalar>::value;
    node["rows"] = val.rows();
    node["cols"] = val.cols();
    return node;
}

template <typename T, std::enable_if_t<!detail::is_eigen<T>::value, int> = 0>
YAML::Node format(const T& val)
{
    YAML::Node node;
    node["type"] = detail::get_type<T>::value;
    return node;
}

template <typename T>
YAML::Node format(const T* val, int size)
{
    YAML::Node node;
    node["type"] = detail::get_type<T>::value;
    node["rows"] = size;
    return node;
}

YAML::Node format(const UTCTime& time);
YAML::Node format(const ephemeris::GPSEphemeris& eph);
YAML::Node format(const ephemeris::GalileoEphemeris& eph);
YAML::Node format(const ephemeris::GlonassEphemeris& eph);
YAML::Node format(const meas::ImuSample& imu);
YAML::Node format(const meas::GnssObservation& obs);
YAML::Node format(const math::Quat<double>& q);
YAML::Node format(const math::DQuat<double>& dq);
YAML::Node format(const math::Jet<double>& dq);
YAML::Node format(const math::TwoJet<double>& dq);

template <typename... Args, size_t... I>
YAML::Node __make_format_impl(const std::array<std::string, sizeof...(Args)>& names,
                              std::index_sequence<I...>,
                              const Args&... args)
{
    YAML::Node out;
    std::stringstream ss;
    const auto fun = [&](const std::string& name, YAML::Node node) {
        out[name] = node;
        return 1;
    };
    int dummy[] = {fun(names[I], format(args))...};
    (void)dummy;
    return out;
}

template <typename... Args>
YAML::Node makeFormat(const std::array<std::string, sizeof...(Args)>& names)
{
    return __make_format_impl(names, std::index_sequence_for<Args...>{}, Args()...);
}

};  // namespace logging
}  // namespace mc
