#pragma once

#include <cstdint>

struct GnssID
{
    enum
    {
        GPS = 0,
        SBAS = 1,
        Galileo = 2,
        Beidou = 3,
        Qzss = 5,
        Glonass = 6
    };
};

template <typename T>
T sign(T in)
{
    return static_cast<T>((in >= 0) - (in < 0));
}
