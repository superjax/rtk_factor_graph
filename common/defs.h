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
