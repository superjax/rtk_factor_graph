#pragma once

#include <cmath>
#include <cstdint>

namespace mc {

constexpr double C_LIGHT = 299792458.0;                 // speed of light (m/s)
static constexpr double GM_EARTH = 3.986005e14;         // Mass of the earth (kg)
static constexpr double OMEGA_EARTH = 7.2921151467e-5;  // Angular Velocity of the earth (rad/s)

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

enum class JacobianSide
{
    LEFT,
    RIGHT
};

template <typename T>
T sign(T in)
{
    return static_cast<T>((in >= 0) - (in < 0));
}

template <typename T>
T sat(const T& in, const T& min, const T& max)
{
    return (in > max) ? max : (in < min) ? min : in;
}

constexpr double deg2Rad(const double deg)
{
    return M_PI * deg / 180.0;
}

constexpr double rad2Deg(const double rad)
{
    return 180.0 * rad / M_PI;
}

}  // namespace mc
