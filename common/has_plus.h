
namespace mc {

template <typename T, typename Tp>
struct has_plus
{
 private:
    template <typename U, typename Up>
    static auto test(int)
        -> decltype(std::declval<U>().operator+(std::declval<Up>()), std::true_type());

    template <typename U, typename Up>
    static std::false_type test(...);

 public:
    static constexpr bool value = std::is_same<decltype(test<T, Tp>(0)), std::true_type>::value;
};

template <typename T, typename T2 = T>
struct has_minus
{
 private:
    template <typename U, typename U2>
    static auto test(int)
        -> decltype(std::declval<U>().operator-(std::declval<U2>()), std::true_type());

    template <typename U, typename U2>
    static std::false_type test(...);

 public:
    static constexpr bool value = std::is_same<decltype(test<T, T2>(0)), std::true_type>::value;
};

}  // namespace mc
