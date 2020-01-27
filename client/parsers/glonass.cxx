#include "client/parsers/glonass.h"
#include "client/parsers/bit_tools.h"
#include "common/print.h"
#include "utils/crc.h"

namespace mc {
namespace client {
namespace parsers {

constexpr double GlonassEphemeris::FREQ1_GLO;
constexpr double GlonassEphemeris::DFRQ1_GLO;
constexpr double GlonassEphemeris::FREQ2_GLO;
constexpr double GlonassEphemeris::DFRQ2_GLO;
constexpr double GlonassEphemeris::FREQ3_GLO;

GlonassEphemeris::GlonassEphemeris()
{
    gnssID = GnssID::Glonass;
    sat = 255;
}

GlonassEphemeris::GlonassEphemeris(int sat_id)
{
    gnssID = GnssID::Glonass;
    sat = sat_id;
}

bool GlonassEphemeris::parse(const uint8_t* msg, const size_t size)
{
    if (size < 16)
    {
        dbg("Not enough bytes in GLONASS string");
        return false;
    }

    // Switch Byte Order
    uint8_t buf[64];
    const uint8_t* p = msg;
    int i = 0;
    for (int k = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            buf[k++] = p[3 - j];
        }
        p += 4;
    }

    // Check the Hamming Code
    if (!utils::gloTest(buf))
    {
        dbg("GLONASS failed Hamming code check");
        return false;
    }

    int frame_num = getBit<4>(buf, 1);

    // We only care about strings [1-4]
    if (frame_num < 1 || 5 < frame_num)
    {
        return false;
    }

    // reset the parser if the time mark (MB) has changed
    int time_mark = (uint16_t)(buf[12] << 8) | buf[13];
    if (time_mark != time_mark_)
    {
        frame_flags_ = 0;
    }
    time_mark_ = time_mark;

    switch (frame_num)
    {
    case 1:
        frame1(buf);
        break;
    case 2:
        frame2(buf);
        break;
    case 3:
        frame3(buf);
        break;
    case 4:
        frame4(buf);
        break;
    case 5:
        frame5(buf);
        break;
    default:
        return false;
    }

    // Wait to get all frames
    if (frame_flags_ != (FRAME1 | FRAME2 | FRAME3 | FRAME4 | FRAME5))
    {
        return false;
    }

    // Only spit out ephemeris on the fifth frame, so we don't spam too much
    if (frame_num != 5)
    {
        return false;
    }

    return true;
}

bool GlonassEphemeris::frame1(const uint8_t* buf)
{
    // dbg("Gal frame 1");
    // int P1 = getBit<2>(buf, 7);
    tk_h_ = getBit<5>(buf, 9);
    tk_m_ = getBit<6>(buf, 14);
    tk_s_ = getBit<1>(buf, 20) * 30;
    vel[0] = getBitGlo<24>(buf, 21) * P2_20 * 1E3;
    acc[0] = getBitGlo<5>(buf, 45) * P2_30 * 1E3;
    pos[0] = getBitGlo<27>(buf, 50) * P2_11 * 1E3;
    frame_flags_ |= FRAME1;
    return convertTime();
}

bool GlonassEphemeris::frame2(const uint8_t* buf)
{
    // dbg("Gal frame 2");
    svh = getBit<3>(buf, 5);
    // int P2 = getBit<1>(buf, 8);
    tb_ = getBit<7>(buf, 9);
    iode = tb_;
    vel[1] = getBitGlo<24>(buf, 21) * P2_20 * 1E3;
    acc[1] = getBitGlo<5>(buf, 45) * P2_30 * 1E3;
    pos[1] = getBitGlo<27>(buf, 50) * P2_11 * 1E3;

    frame_flags_ |= FRAME2;
    return convertTime();
}

bool GlonassEphemeris::frame3(const uint8_t* buf)
{
    // dbg("Gal frame 3");
    // int P3 = getBit<1>(buf, 5);
    gamn = getBitGlo<11>(buf, 6) * P2_40;
    // int P = getBit<2>(buf, 18);
    // int ln = getBit<1>(buf, 20);
    vel[2] = getBitGlo<24>(buf, 21) * P2_20 * 1E3;
    acc[2] = getBitGlo<5>(buf, 45) * P2_30 * 1E3;
    pos[2] = getBitGlo<27>(buf, 50) * P2_11 * 1E3;
    frame_flags_ |= FRAME3;
    return true;
}

bool GlonassEphemeris::frame4(const uint8_t* buf)
{
    // dbg("Gal frame 4");
    taun = getBitGlo<22>(buf, 5) * P2_30;
    dtaun = getBitGlo<5>(buf, 27) * P2_30;
    age = getBit<5>(buf, 32);
    // int P4 = getBit<1>(buf, 51);
    sva = getBit<4>(buf, 52);
    NT_ = getBit<11>(buf, 59);
    slot = getBit<5>(buf, 70);
    // int M = getBit<2>(buf, 75);
    frame_flags_ |= FRAME4;
    return true;
}

bool GlonassEphemeris::frame5(const uint8_t* buf)
{
    // dbg("Gal Frame 5");
    NA_ = getBit<11>(buf, 5);
    // int tau_c = getBit<32>(buf, 16);
    N4_ = getBit<5>(buf, 49);
    // int tau_ops = getBit<22>(buf, 54);
    // int l_n = getBit<1>(buf, 76);
    frame_flags_ |= FRAME5;

    convertTime();
    return true;
}

bool GlonassEphemeris::convertTime()
{
    if (!got(FRAME4) || !got(FRAME5))
        return false;

    static UTCTime Jan1_1996 = UTCTime::fromCalendar(1996, 1, 1, 0, 0, 0);
    UTCTime start_of_year =
        Jan1_1996 + (N4_ - 1) * (3 * UTCTime::SEC_IN_YEAR + UTCTime::SEC_IN_LEAP_YEAR) - (3 * 3600);
    UTCTime start_of_day = start_of_year + (NA_ - 1) * UTCTime::SEC_IN_DAY;

    // Convert t_k (tof) to UTC
    if (got(FRAME1))
    {
        tof = start_of_day + (tk_h_ * 3600 + tk_m_ * 60 + tk_s_);
    }

    // Convert t_b (toe) to UTC
    if (got(FRAME2))
    {
        toe = start_of_day + (tb_ * 60 * 15);
    }

    return (got(FRAME1) && got(FRAME2));
}
}  // namespace parsers
}  // namespace client
}  // namespace mc
