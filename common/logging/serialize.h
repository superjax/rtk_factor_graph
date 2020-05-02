#pragma once

#include "common/logging/logger.h"

namespace mc {
namespace logging {

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

template <typename T>
void serialize(Logger& log, const T& arg)
{

}

}  // namespace logging
}  // namespace mc
