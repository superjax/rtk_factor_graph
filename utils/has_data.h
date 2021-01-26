#pragma once

namespace mc {

namespace utils {

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

}  // namespace utils

}  // namespace mc
