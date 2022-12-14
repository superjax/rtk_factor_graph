#pragma once

#include <random>

#include "common/defs.h"
#include "common/utctime.h"

namespace mc {
namespace meas {

class GnssObservation
{
 public:
    UTCTime t;
    uint8_t gnss_id;
    uint8_t sat_num;
    double freq;  // frequency (Hz)

    double pseudorange;    // (m)
    double doppler;        // (m/s)
    double carrier_phase;  // (cycles)

    static GnssObservation Random()
    {
        GnssObservation out;
        out.setRandom();
        return out;
    }

    inline void setRandom()
    {
        std::normal_distribution<double> normal(1, 0);
        std::default_random_engine gen;

        t = UTCTime(0, 0);
        gnss_id = GnssID::GPS;
        sat_num = 0;
        pseudorange = 25'000'000 + 10'000 * normal(gen);
        doppler = 1'000 * normal(gen);
        carrier_phase = pseudorange / 0.19 + normal(gen) * 1'000;
        freq = 0;
    }

    inline void setZero()
    {
        t = UTCTime(0, 0);
        gnss_id = GnssID::GPS;
        sat_num = 0;
        pseudorange = 0;
        doppler = 0;
        carrier_phase = 0;
        freq = 0;
    }
};

}  // namespace meas
}  // namespace mc
