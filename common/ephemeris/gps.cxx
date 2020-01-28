#include "common/ephemeris/gps.h"
#include "common/ephemeris/bit_tools.h"
#include "common/print.h"

namespace mc {
namespace ephemeris {

GPSEphemeris::GPSEphemeris(int sat_id)
{
    gnssID = GnssID::GPS;
    sat = sat_id;
}

bool GPSEphemeris::parse(const uint8_t* buf, size_t size)
{
    if (size < 40)
    {
        dbg("Incomplete GPS Frame");
        return false;
    }

    // Flip the bits and strip them of parity
    uint8_t subframe[30];
    const uint8_t* p = buf;
    for (int i = 0; i < 10; ++i)
    {
        setBit<24>(subframe, i * 24, *(reinterpret_cast<const uint32_t*>(p)) >> 6);
        p += 4;
    }

    int id = getBit<3>(subframe, 43);
    if (id < 1 || id > 3)
    {
        return false;
    }

    switch (id)
    {
    case 1:
        frame1(subframe);
        break;
    case 2:
        frame2(subframe);
        break;
    case 3:
        frame3(subframe);
        break;
    default:
        return false;
    }

    // ensure we have seen all the subframes
    if (collected_subframes != (FRAME1 | FRAME2 | FRAME3))
    {
        dbg("Waiting for Subframes");
        return false;
    }

    // Ensure that the iode is the same across all frames (and for the clock)
    else if (iode1 != iode2 || iode1 != iode3 || iode != (iodc & 0xFF))
    {
        dbg("mis-matched issue-of-data %d, %d, %d", iode1, iode2, iode3);
        return false;
    }

    dbg("Got all subframes");

    // Set toe and toc
    toe = UTCTime::fromGPS(week, toes * 1000);
    toc = UTCTime::fromGPS(week, tocs * 1000);

    // Reset synchronization
    iode1 = iode2 = iode3 = 0;

    return true;
}

bool GPSEphemeris::frame1(const uint8_t* buf)
{
    if (buf[0] != 0x8B)
    {
        dbg("GPS Preamble Incorrect");
        return false;
    }
    dbg("GOT FRAME 1");
    // double tow = getBit<17>(buf, 24) * 6.0;
    int week_mod = getBit<10>(buf, 48);  // This is the week modulo 2^10
    code_on_L2 = getBit<2>(buf, 58);
    ura = getBit<4>(buf, 60);
    health = getBit<6>(buf, 64);
    uint8_t iodc0 = getBit<2>(buf, 70);
    L2_P_data_flag = getBit<1>(buf, 72);
    int _tgd = getBit<8, Signed>(buf, 160);
    iode = getBit<8>(buf, 168);
    tocs = getBit<16>(buf, 176) * 16.0;
    af2 = getBit<8, Signed>(buf, 192) * P2_55;
    af1 = getBit<16, Signed>(buf, 200) * P2_43;
    af0 = getBit<22, Signed>(buf, 216) * P2_31;

    alert_flag = 0;  // TODO: populate

    tgd = _tgd == -128 ? 0.0 : _tgd * P2_31;
    iodc = (iodc0 << 8) + iode;
    week = week_mod + UTCTime::GPS_WEEK_ROLLOVER;

    iode1 = iode;
    collected_subframes |= FRAME1;

    return true;
}

bool GPSEphemeris::frame2(const uint8_t* buf)
{
    if (buf[0] != 0x8B)
    {
        dbg("GPS Preamble Incorrect");
        return false;
    }
    dbg("GOT FRAME 2");

    iode = getBit<8>(buf, 48);
    crs = getBit<16, Signed>(buf, 56) * P2_5;
    delta_n = getBit<16, Signed>(buf, 72) * P2_43 * PI;
    m0 = getBit<32, Signed>(buf, 88) * P2_31 * PI;
    cuc = getBit<16, Signed>(buf, 120) * P2_29;
    ecc = getBit<32>(buf, 136) * P2_33;
    cus = getBit<16, Signed>(buf, 168) * P2_29;
    sqrta = getBit<32>(buf, 184) * P2_19;
    toes = getBit<16>(buf, 216) * 16.0;
    fit_interval_flag = getBit<1>(buf, 232) ? 0.0 : 4.0;

    iode2 = iode;
    collected_subframes |= FRAME2;

    return true;
}

bool GPSEphemeris::frame3(const uint8_t* buf)
{
    if (buf[0] != 0x8B)
    {
        dbg("GPS Preamble Incorrect");
        return false;
    }
    dbg("GOT FRAME 3");
    cic = getBit<16, Signed>(buf, 48) * P2_29;
    omega0 = getBit<32, Signed>(buf, 64) * P2_31 * PI;
    cis = getBit<16, Signed>(buf, 96) * P2_29;
    i0 = getBit<32, Signed>(buf, 112) * P2_31 * PI;
    crc = getBit<16, Signed>(buf, 144) * P2_5;
    w = getBit<32, Signed>(buf, 160) * P2_31 * PI;
    omegadot = getBit<24, Signed>(buf, 192) * P2_43 * PI;
    iode3 = getBit<8>(buf, 216);
    idot = getBit<14, Signed>(buf, 224) * P2_43 * PI;

    iode3 = iode;
    collected_subframes |= FRAME3;

    return true;
}
}  // namespace ephemeris
}  // namespace mc
