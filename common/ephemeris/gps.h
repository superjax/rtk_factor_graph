#pragma once

#include "common/ephemeris/eph.h"

namespace mc {
namespace ephemeris {

class GPSEphemeris : public KeplerianEphemeris
{
 public:
    explicit GPSEphemeris(int sat_id);
    uint8_t health;      // 6 bit health parameter; 0 if healthy; unhealth othersize [0=healthy]
    uint8_t alert_flag;  // 1 = URA may be worse than indicated [0,1]
    uint8_t anti_spoof;  // anti-spoof flag from 0=off; 1=on [0,1]
    uint8_t code_on_L2;  // 0=reserved; 1=P code on L2; 2=C/A on L2 [0,1,2]
    uint8_t ura;         // User Range Accuracy lookup code; 0 is excellent; 15 is use at own risk
                         // [0-15]; see p. 83 GPSICD200C
    uint8_t L2_P_data_flag;       // flag indicating if P is on L2 1=true [0,1]
    uint8_t fit_interval_flag;    // fit interval flag (four hour interval or longer) 0=4 fours;
                                  // 1=greater         [0,1]
    uint16_t age_of_data_offset;  // age of data offset [s]
    double tgd;                   // group delay [s]

    bool parse(const uint8_t* buf, size_t size);

 private:
    bool frame1(const uint8_t* buf);
    bool frame2(const uint8_t* buf);
    bool frame3(const uint8_t* buf);

    // Synchronization Variables
    uint8_t iode1 = 0;
    uint8_t iode2 = 0;
    uint8_t iode3 = 0;
    enum
    {
        FRAME1 = 0x01,
        FRAME2 = 0x02,
        FRAME3 = 0x04,
    };
    uint8_t collected_subframes = 0x00;
};

}  // namespace ephemeris
}  // namespace mc
