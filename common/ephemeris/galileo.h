#pragma once

#include "common/ephemeris/eph.h"

namespace mc {
namespace ephemeris {

class GalileoEphemeris : public KeplerianEphemeris
{
 public:
    explicit GalileoEphemeris(int sat_id);

    bool parse(const uint8_t* buf, size_t size);

    double ai0;
    double ai1;
    double ai2;
    double bgd_e1_e5a;
    double bgd_e1_e5b;
    uint8_t e5b_hs;
    uint8_t e1b_hs;
    uint8_t e5b_dvs;
    uint8_t e1b_dvs;

 private:
    bool frame0(const uint8_t* buf);
    bool frame1(const uint8_t* buf);
    bool frame2(const uint8_t* buf);
    bool frame3(const uint8_t* buf);
    bool frame4(const uint8_t* buf);
    bool frame5(const uint8_t* buf);

    inline bool got(uint8_t frame) const { return collected_subframes & frame; }

    // Synchronization Variables
    uint8_t collected_subframes = 0x00;
    enum
    {
        FRAME0 = 0x01,
        FRAME1 = 0x02,
        FRAME2 = 0x04,
        FRAME3 = 0x08,
        FRAME4 = 0x10,
        FRAME5 = 0x20,
    };

    int time_f;
    int iod[4] = {0};
};
}  // namespace ephemeris
}  // namespace mc
