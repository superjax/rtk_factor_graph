#pragma once

#include <cmath>
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

constexpr double deg2Rad(const double deg)
{
    return M_PI * deg / 180.0;
}

constexpr double rad2Deg(const double rad)
{
    return 180.0 * rad / M_PI;
}
