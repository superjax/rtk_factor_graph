#pragma once

#include <cstdlib>
#include <ostream>

namespace mc {

class Error
{
 private:
    const char* what_;
    constexpr Error(const char* _what) : what_(_what) {}
    constexpr Error() : what_(nullptr) {}

    static constexpr int length(const char* str) { return *str ? 1 + length(str + 1) : 0; }

    template <size_t N, size_t... Idx>
    static constexpr std::array<char, std::index_sequence<Idx...>::size()> toArrayImpl(
        char const (&str)[N],
        std::index_sequence<Idx...>)
    {
        return std::array<char, std::index_sequence<Idx...>::size()>({str[Idx]...});
    }

    template <size_t N, typename Indices = std::make_index_sequence<N>>
    static constexpr std::array<char, Indices::size()> toArray(char const (&str)[N])
    {
        return toArrayImpl(str, Indices());
    }

 public:
    template <size_t N>
    constexpr static Error create(const char (&_what)[N])
    {
        // Converting the char[] -> std::array guarantees that _what is a constexpr
        constexpr size_t len = sizeof(decltype(toArray(_what)));
        static_assert(len > 0u, "cannot create error with empty message");
        return Error(_what);
    }
    static Error none() { return Error(nullptr); }

    inline bool operator==(const Error& other) const
    {
        return reinterpret_cast<size_t>(what_) == reinterpret_cast<size_t>(other.what_);
    }

    inline bool operator!=(const Error& other) const
    {
        return reinterpret_cast<size_t>(what_) != reinterpret_cast<size_t>(other.what_);
    }

    inline bool operator==(const char* _what) const
    {
        return reinterpret_cast<size_t>(what_) == reinterpret_cast<size_t>(_what);
    }

    inline bool operator!=(const char* _what) const
    {
        return reinterpret_cast<size_t>(what_) != reinterpret_cast<size_t>(_what);
    }

    inline bool ok() const { return what_ == nullptr; }
    const char* what() const { return what_; }
};

inline std::ostream& operator<<(std::ostream& os, const Error& err)
{
    os << err.what();
    return os;
}

}  // namespace mc
