#pragma once

#include <cstdint>
#include <limits>

enum
{
    SwitchByteOrder,
    KeepByteOrder,
    Signed,
    Unsigned
};

// clang-format off
template <int Sign, size_t len>
struct TypeHelper {
    typedef
        typename std::conditional<Sign == Unsigned && len <= 8 , uint8_t,
        typename std::conditional<Sign == Unsigned && len <= 16, uint16_t,
        typename std::conditional<Sign == Unsigned && len <= 32, uint32_t,
        typename std::conditional<Sign == Unsigned && len <= 64, uint64_t,
        typename std::conditional<Sign == Signed && len <= 8 , int8_t,
        typename std::conditional<Sign == Signed && len <= 16, int16_t,
        typename std::conditional<Sign == Signed && len <= 32, int32_t,
        typename std::conditional<Sign == Signed && len <= 64, int64_t,
        void>::type>::type>::type>::type>::type>::type>::type>::type type;
};
// clang-format on

template <int len, int Sign = Unsigned, int byteOrder = SwitchByteOrder>
typename TypeHelper<Sign, len>::type getBit(const unsigned char *buf, const int pos)
{
    typedef typename TypeHelper<Unsigned, len + 8>::type tempType;
    typedef typename TypeHelper<Unsigned, len>::type UnsignedRetType;
    typedef typename TypeHelper<Sign, len>::type retType;
    static constexpr tempType mask = len == 64 ? 0xFFFFFFFF : (1ul << len) - 1ul;
    const size_t last_bit = pos + len - 1;
    static_assert(len <= 64 && len > 0);

    const uint8_t *p = buf + (pos / 8);

    tempType tmp = *p;
    UnsignedRetType ret;
    if constexpr (byteOrder == SwitchByteOrder)
    {
        for (int i = 8 - (pos & 7); i < len; i += 8)
        {
            tmp <<= 8;
            tmp |= *(++p);
        }
        ret = ((tmp >> (7 - (last_bit & 7))) & mask);
    }
    else
    {
        for (int i = 0; i < len; i += 8)
        {
            tmp |= (*(++p) << (i + 8));
        }
        ret = ((tmp >> (pos % 8)) & mask);
    }

    // If requested, move the sign bit to the end of the type
    if constexpr (Sign == Signed)
    {
        static constexpr tempType one = 1;
        if (ret & (one << (len - 1)))
        {
            return (retType)(ret | (std::numeric_limits<tempType>::max() << len));
        }
    }
    return ret;
}

template <int Sign = Unsigned, int byteOrder = SwitchByteOrder>
typename TypeHelper<Sign, 32>::type getBit(const unsigned char *buf, const int pos, const int len)
{
    typedef typename TypeHelper<Unsigned, 64>::type tempType;
    typedef typename TypeHelper<Unsigned, 32>::type UnsignedRetType;
    typedef typename TypeHelper<Sign, 32>::type retType;
    tempType mask = len == 64 ? 0xFFFFFFFF : (1ul << len) - 1ul;
    const size_t last_bit = pos + len - 1;
    // assert(len <= 64 && len > 0);

    const uint8_t *p = buf + (pos / 8);

    tempType tmp = *p;
    UnsignedRetType ret;
    if constexpr (byteOrder == SwitchByteOrder)
    {
        for (int i = 8 - (pos & 7); i < len; i += 8)
        {
            tmp <<= 8;
            tmp |= *(++p);
        }
        ret = ((tmp >> (7 - (last_bit & 7))) & mask);
    }
    else
    {
        for (int i = 0; i < len; i += 8)
        {
            tmp |= (*(++p) << (i + 8));
        }
        ret = ((tmp >> (pos % 8)) & mask);
    }

    // If requested, move the sign bit to the end of the type
    if constexpr (Sign == Signed)
    {
        static constexpr tempType one = 1;
        if (ret & (one << (len - 1)))
        {
            return (retType)(ret | (std::numeric_limits<tempType>::max() << len));
        }
    }
    return ret;
}

template <int len, int Sign = Unsigned, int byteOrder = SwitchByteOrder>
void setBit(uint8_t *buf, const int pos, const unsigned data)
{
    static_assert(len <= 64 && len > 0);
    static_assert(byteOrder == SwitchByteOrder, "KeepByteOrder Not implemented");
    typedef typename TypeHelper<Unsigned, len + 8>::type tempType;
    static constexpr tempType mask = len == 64 ? 0xFFFFFFFF : (1ul << len) - 1ul;
    const size_t last_bit = pos + len - 1;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Winit-self"
#ifndef __clang_analyzer__
    tempType m = data & mask;
    tempType wmask = ~(mask << (7 - (last_bit & 7)));
    m <<= (7 - (last_bit & 7));
    uint8_t *p = buf + last_bit / 8;
    int i = (last_bit & 7) + 1;
    (*p &= wmask) |= m;
    while (i < len)
    {
        m >>= 8;
        wmask >>= 8;
        (*(--p) &= wmask) |= m;
        i += 8;
    }
#endif
#pragma GCC diagnostic pop
}

template <int Sign = Unsigned, int byteOrder = SwitchByteOrder>
void setBit(unsigned char *buff, const int pos, const unsigned data, const int len)
{
    static_assert(byteOrder == SwitchByteOrder, "KeepByteOrder Not implemented");
    // static_assert(len <= 64 && len > 0);
    typedef typename TypeHelper<Unsigned, 64>::type tempType;
    tempType mask = len == 64 ? 0xFFFFFFFF : (1ul << len) - 1ul;
    size_t last_bit = pos + len - 1;

    tempType m = data & mask;
    tempType wmask = ~(mask << (7 - (last_bit & 7)));
    m <<= (7 - (last_bit & 7));
    uint8_t *p = buff + last_bit / 8;
    int i = (last_bit & 7) + 1;
    (*p &= wmask) |= m;
    while (i < len)
    {
        m >>= 8;
        wmask >>= 8;
        (*(--p) &= wmask) |= m;
        i += 8;
    }
}

template <int len>
double getBitGlo(const uint8_t *buf, const int pos)
{
    const double val = getBit<len - 1>(buf, pos + 1);
    return getBit<1>(buf, pos) ? -val : val;
}
