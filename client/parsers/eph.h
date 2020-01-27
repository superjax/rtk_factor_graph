#pragma once

#include <cstdint>

#include "common/defs.h"
#include "common/utctime.h"

namespace mc {
namespace client {
namespace parsers {

class EphBase
{
 public:
    static constexpr double PI = 3.1415926535897932;
    static constexpr double P2_5 = 0.03125;                 // 2^-5
    static constexpr double P2_11 = 4.882812500000000E-04;  // 2^-11
    static constexpr double P2_19 = 1.907348632812500E-06;  // 2^-19
    static constexpr double P2_20 = 9.536743164062500E-07;  // 2^-20
    static constexpr double P2_29 = 1.862645149230957E-09;  // 2^-29
    static constexpr double P2_30 = 9.313225746154785E-10;  // 2^-30
    static constexpr double P2_31 = 4.656612873077393E-10;  // 2^-31
    static constexpr double P2_32 = 2.328306436538696E-10;  // 2^-32
    static constexpr double P2_33 = 1.164153218269348E-10;  // 2^-33
    static constexpr double P2_34 = 5.820766091346740E-11;  // 2^-34
    static constexpr double P2_40 = 9.094947017729280E-13;  // 2^-40
    static constexpr double P2_43 = 1.136868377216160E-13;  // 2^-43
    static constexpr double P2_46 = 1.421085471520200E-14;  // 2^-46
    static constexpr double P2_55 = 2.775557561562891E-17;  // 2^-55
    static constexpr double P2_59 = 1.734723475976810E-18;  // 2^-59

    uint8_t gnssID;
    uint8_t sat;

    std::string Type() const
    {
        switch (gnssID)
        {
        case GnssID::GPS:
            return "GPS";
        case GnssID::Galileo:
            return "Galileo";
        case GnssID::Glonass:
            return "Glonass";
        case GnssID::Qzss:
            return "Qzss";
        case GnssID::Beidou:
            return "Beidou";
        default:
            return "Unknown";
        }
    }
};

class KeplerianEphemeris : public EphBase
{
 public:
    UTCTime toe;  // reference time ephemeris (UTC Time) [s]
    UTCTime toc;  // reference time (clock)   (UTC Time) [s]

    uint32_t tow;   // time of week in subframe1; the time of the leading bit edge of subframe 2 [s]
    uint16_t iodc;  // 10 bit issue of data (clock); 8 LSB bits will match the iode []
    uint8_t iode;   // 8 bit  issue of data (ephemeris) []
    uint16_t week;  // 10 bit gps week 0-1023 (user must account for week rollover ) [week]
    uint32_t toes;  // Time of ephemeris (seconds part)
    uint32_t tocs;  // Time of clock (seconds part)

    double af2;       // polynomial clock correction coefficient (rate of clock drift) [s/s^2]
    double af1;       // polynomial clock correction coefficient (clock drift) [s/s]
    double af0;       // polynomial clock correction coefficient (clock bias) [s]
    double m0;        // mean anomaly at reference time [rad]
    double delta_n;   // mean motion difference from computed value [rad/s]
    double ecc;       // eccentricity []
    double sqrta;     // square root of the semi-major axis [m^(1/2)]
    double omega0;    // longitude of ascending node of orbit plane at weekly epoch [rad]
    double i0;        // inclination angle at reference time [rad]
    double w;         // argument of perigee [rad]
    double omegadot;  // rate of right ascension [rad/s]
    double idot;      // rate of inclination angle [rad/s]
    double cuc;  // amplitude of the cosine harmonic correction term to the argument of latitude
                 // [rad]
    double cus;  // amplitude of the sine harmonic correction term to the argument of latitude [rad]
    double crc;  // amplitude of the cosine harmonic correction term to the orbit radius [m]
    double crs;  // amplitude of the sine harmonic correction term to the orbit radius [m]
    double cic;  // amplitude of the cosine harmonic correction term to the angle of inclination
                 // [rad]
    double cis;  // amplitude of the sine harmonic correction term to the angle of inclination [rad]
};
}  // namespace parsers
}  // namespace client
}  // namespace mc
