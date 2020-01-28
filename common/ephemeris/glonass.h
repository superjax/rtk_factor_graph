#pragma once

#include "common/ephemeris/eph.h"
#include "common/matrix_defs.h"

namespace mc {
namespace ephemeris {

class GlonassEphemeris : public EphBase
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 public:
    GlonassEphemeris();
    explicit GlonassEphemeris(int sat_id);
    static constexpr double FREQ1_GLO = 1.60200e9;   // GLONASS G1 base frequency (Hz)
    static constexpr double DFRQ1_GLO = 0.56250E6;   // GLONASS G1 bias frequency (Hz/n)
    static constexpr double FREQ2_GLO = 1.24600e9;   // GLONASS G2 base frequency (Hz)
    static constexpr double DFRQ2_GLO = 0.43750E6;   // GLONASS G2 bias frequency (Hz/n)
    static constexpr double FREQ3_GLO = 1.202025e9;  // GLONASS G3 frequency (Hz)
    int iode;                                        // IODE (0-6 bit of tb field)
    int slot;                                        // satellite frequency slot
    int svh, sva, age;                               // satellite health, accuracy, age of operation
    UTCTime toe;                                     // epoch of epherides (UTC)
    UTCTime tof;                                     // message frame time (UTC)
    Vec3 pos;                                        // satellite position (ecef) (m)
    Vec3 vel;                                        // satellite velocity (ecef) (m/s)
    Vec3 acc;                                        // satellite acceleration (ecef) (m/s^2)
    double taun, gamn;                               // SV clock bias (s)/relative freq bias
    double dtaun;                                    // delay between L1 and L2 (s)

    bool parse(const uint8_t* buf, size_t size);

 private:
    bool frame1(const uint8_t* buf);
    bool frame2(const uint8_t* buf);
    bool frame3(const uint8_t* buf);
    bool frame4(const uint8_t* buf);
    bool frame5(const uint8_t* buf);

    // Synchronization Variables
    enum
    {
        FRAME1 = 0x01,
        FRAME2 = 0x02,
        FRAME3 = 0x04,
        FRAME4 = 0x08,
        FRAME5 = 0x10,
    };

    bool convertTime();

    bool got(int frame) { return frame_flags_ & frame; }
    uint8_t frame_flags_ = 0x00;
    int time_mark_ = 0;  // MB Field

    //  is  the  time  referenced  to  the  beginning  of  the  frame  within  the  current  day. It
    //  is calculated  according  to  the  satellite  time  scale
    int tk_h_;  // is  the  time  referenced  to  the  beginning  of  the  frame  within  the
                // current  day.  It  is calculated  according  to  the  satellite  time  scale
    int tk_m_;  // integer  number  of  minutes  elapsed since the beginning of the current hour
    int tk_s_;  // number of thirty-second intervals elapsed since the beginning of the current day
    int tb_;    // an index of a time interval within current day according to UTC(SU) + 03 hours 00
                // min.
    int NT_;  // current date, calendar number of day within four-year interval starting from aleap
              // year
    int N4_;  // four-year interval number starting from 1996
    int NA_;  // calendar day number within the four-year period beginning since the leap year
};
}  // namespace ephemeris
}  // namespace mc
