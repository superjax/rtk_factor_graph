#include "common/ephemeris/galileo.h"
#include "common/check.h"
#include "common/ephemeris/bit_tools.h"
#include "common/print.h"
#include "utils/crc.h"

namespace mc {
namespace ephemeris {

constexpr double GalileoEphemeris::FREQUENCY_E1;
constexpr double GalileoEphemeris::FREQUENCY_E5b;
constexpr double GalileoEphemeris::FREQUENCY_E5a;
constexpr double GalileoEphemeris::FREQUENCY_E5ab;
constexpr double GalileoEphemeris::FREQUENCY_E6;

constexpr double GalileoEphemeris::LAMBDA_E1;
constexpr double GalileoEphemeris::LAMBDA_E5b;
constexpr double GalileoEphemeris::LAMBDA_E5ab;
constexpr double GalileoEphemeris::LAMBDA_E5a;
constexpr double GalileoEphemeris::LAMBDA_E6;

GalileoEphemeris::GalileoEphemeris(int sat_id)
{
    gnssID = GnssID::Galileo;
    sat = sat_id;
}

double GalileoEphemeris::getWavelength(int frequency) const
{
    switch (frequency)
    {
    case E1:
        return LAMBDA_E1;
    case E5b:
        return LAMBDA_E5b;
    case E5ab:
        return LAMBDA_E5ab;
    case E5a:
        return LAMBDA_E5a;
    case E6:
        return LAMBDA_E6;
    }
    check(false, "supplied invalid frequency slot for Galileo {}", fmt(frequency));
    return -1;
}

bool GalileoEphemeris::parse(const uint8_t* msg, size_t size)
{
    (void)size;
    // swap buffer endianness
    uint8_t buf[64];
    const uint8_t* p = msg;
    int k = 0;
    for (int i = 0; i < 8; ++i)
    {
        for (int j = 0; j < 4; j++)
        {
            buf[k++] = p[3 - j];
        }
        p += 4;
    }

    uint8_t part1 = getBit<1>(buf, 0);
    uint8_t page1 = getBit<1>(buf, 1);
    uint8_t part2 = getBit<1>(buf + 16, 0);
    uint8_t page2 = getBit<1>(buf + 16, 1);

    /* skip alert page */
    if (page1 == 1 || page2 == 1)
    {
        dbg("Skipping alert page");
        return 0;
    }

    // Check the even-odd page
    if (part1 != 0 || part2 != 1)
    {
        dbg("GAL page even/odd error: sat={}", fmt(sat));
        return false;
    }

    // CRC check (offset by four bytes, so shift it over)
    uint8_t crc_buff[26] = {0};
    int j = 4;
    for (int i = 0; i < 15; ++i)
    {
        setBit<8>(crc_buff, j, buf[i]);
        j += 8;
    }
    j = 118;
    for (int i = 0; i < 11; i++)
    {
        setBit<8>(crc_buff, j, buf[16 + i]);
        j += 8;
    }
    if (utils::crc24(crc_buff, 25) != getBit<24>(buf + 16, 82))
    {
        dbg("Galileo CRC Error: sat={}", fmt(sat));
        return false;
    }

    int type = getBit<6>(buf, 2);  // INAV Page Type

    // We only care about Ephemeris, Ionosphere Corrections, and UTC conversion
    if (type > 6)
    {
        return false;
    }

    // restart collection flags
    if (type == 2)
    {
        collected_subframes = 0x00;
    }

    for (int i = 0; i < 14; i++)
    {
        // Get the first 112 bits off the even page
        // Offset by 2 to make it easy to compare with ICD
        buf[i] = getBit<8>(buf, i * 8 + 2);
    }

    for (int i = 14; i < 16; i++)
    {
        // Get the remaining 16 bits from the odd page
        buf[i] = getBit<8>(buf, i * 8 + 122);
    }

    // At this point, all 128 data bits have been consolidated in buf
    switch (type)
    {
    case 0:
        frame0(buf);
        break;
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

    if (collected_subframes != (FRAME0 | FRAME1 | FRAME2 | FRAME3 | FRAME4 | FRAME5))
        return false;

    /* test consistency of iod across frames */
    if (iod[0] != iod[1] || iod[0] != iod[2] || iod[0] != iod[3])
    {
        return false;
    }

    // Compute time stamps
    iode = iodc = iod[0];
    UTCTime ttr = UTCTime::fromGalileo(week, tow * 1000.0);
    double dt = (UTCTime::fromGalileo(week, toes * 1000.0) - ttr).toSec();
    if (dt > 302400.0)
        week--;
    else if (dt < -302400.0)
        week++;
    week += 1024;
    toe = UTCTime::fromGalileo(week, toes * 1000);
    toc = UTCTime::fromGalileo(week, tocs * 1000.0);

    return true;
}

bool GalileoEphemeris::frame0(const uint8_t* buf)
{
    int type = getBit<6>(buf, 0);
    time_f = getBit<2>(buf, 6);
    week = getBit<12>(buf, 96);
    tow = getBit<20>(buf, 108);
    if (type != 0)
    {
        dbg("Galileo word type 0 error : expected=0, got {}", fmt(type));
        return false;
    }
    collected_subframes |= FRAME0;
    return true;
}

bool GalileoEphemeris::frame1(const uint8_t* buf)
{
    int type = getBit<6>(buf, 0);
    iod[0] = getBit<10>(buf, 6);
    toes = getBit<14>(buf, 16) * 60.0;
    m0 = getBit<32, Signed>(buf, 30) * P2_31 * M_PI;
    ecc = getBit<32>(buf, 62) * P2_33;
    sqrta = getBit<32>(buf, 94) * P2_19;
    if (type != 1)
    {
        dbg("Galileo word type 1 error : expected=1, got {}", fmt(type));
        return false;
    }
    collected_subframes |= FRAME1;
    return true;
}

bool GalileoEphemeris::frame2(const uint8_t* buf)
{
    int type = getBit<6>(buf, 0);
    iod[1] = getBit<10>(buf, 6);
    omega0 = getBit<32, Signed>(buf, 16) * P2_31 * M_PI;
    i0 = getBit<32, Signed>(buf, 48) * P2_31 * M_PI;
    w = getBit<32, Signed>(buf, 80) * P2_31 * M_PI;
    idot = getBit<14, Signed>(buf, 112) * P2_43 * M_PI;
    if (type != 2)
    {
        dbg("Galileo word type 2 error : expected=2, got {}", fmt(type));
        return false;
    }
    collected_subframes |= FRAME2;
    return true;
}

bool GalileoEphemeris::frame3(const uint8_t* buf)
{
    int type = getBit<6>(buf, 0);
    iod[2] = getBit<10>(buf, 6);
    omegadot = getBit<24, Signed>(buf, 16) * P2_43 * M_PI;
    delta_n = getBit<16, Signed>(buf, 40) * P2_43 * M_PI;
    cuc = getBit<16, Signed>(buf, 56) * P2_29;
    cus = getBit<16, Signed>(buf, 72) * P2_29;
    crc = getBit<16, Signed>(buf, 88) * P2_5;
    crs = getBit<16, Signed>(buf, 104) * P2_5;
    if (type != 3)
    {
        dbg("Galileo word type 3 error : expected=3, got {}", fmt(type));
        return false;
    }
    collected_subframes |= FRAME3;
    return true;
}

bool GalileoEphemeris::frame4(const uint8_t* buf)
{
    int type = getBit<6>(buf, 0);
    iod[3] = getBit<10>(buf, 6);
    int svid = getBit<6>(buf, 16);
    cic = getBit<16, Signed>(buf, 22) * P2_29;
    cis = getBit<16, Signed>(buf, 38) * P2_29;
    tocs = getBit<14>(buf, 54) * 60.0;
    af0 = getBit<31, Signed>(buf, 68) * P2_34;
    af1 = getBit<21, Signed>(buf, 99) * P2_46;
    af2 = getBit<6, Signed>(buf, 120) * P2_59;

    if (type != 4)
    {
        dbg("Galileo word type 4 error : expected=4, got {}", fmt(type));
        return false;
    }
    if (svid != sat)
    {
        dbg("Galileo satellite id doesn't match: expected {}, got {}", fmt(sat, svid));
        return false;
    }
    collected_subframes |= FRAME4;
    return true;
}

bool GalileoEphemeris::frame5(const uint8_t* buf)
{
    int type = getBit<6>(buf, 0);
    bgd_e1_e5a = getBit<10, Signed>(buf, 47) * P2_32;
    bgd_e1_e5b = getBit<10, Signed>(buf, 57) * P2_32;
    e5b_hs = getBit<2>(buf, 67);
    e1b_hs = getBit<2>(buf, 69);
    e5b_dvs = getBit<1>(buf, 70);
    e1b_dvs = getBit<1>(buf, 71);
    // week = getBit<12>(buf, 72);
    // tow = getBit<20>(buf, 84); // Use frame 0 for time

    if (type != 5)
    {
        dbg("Galileo word type 5 error : expected=5, got {}", fmt(type));
        return false;
    }
    collected_subframes |= FRAME5;
    return true;
}

}  // namespace ephemeris
}  // namespace mc
