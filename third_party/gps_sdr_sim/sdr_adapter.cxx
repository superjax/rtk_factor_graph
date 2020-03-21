#include "third_party/gps_sdr_sim/sdr_adapter.h"

namespace third_party {
namespace gps_sdr_sim {

gpstime_t toSdr(const mc::UTCTime& t)
{
    gpstime_t out;
    out.sec = t.GpsTow();
    out.sec = t.week();
    return out;
}

ephem_t toSdr(const mc::ephemeris::GPSEphemeris& eph)
{
    ephem_t out;
    out.toc = toSdr(eph.toc);
    out.toe = toSdr(eph.toe);
    gps2date(&out.toe, &out.t);
    out.iodc = eph.iodc;
    out.iode = eph.iode;
    out.deltan = eph.delta_n;
    out.cuc = eph.cuc;
    out.cus = eph.cus;
    out.cic = eph.cic;
    out.cis = eph.cis;
    out.crc = eph.crc;
    out.crs = eph.crs;
    out.ecc = eph.ecc;
    out.sqrta = eph.sqrta;
    out.m0 = eph.m0;
    out.omg0 = eph.omega0;
    out.inc0 = eph.i0;
    out.aop = eph.w;
    out.omgdot = eph.omegadot;
    out.idot = eph.idot;
    out.af0 = eph.af0;
    out.af1 = eph.af1;
    out.af2 = eph.af2;
    out.tgd = eph.tgd;
    out.svhlth = eph.health;
    out.codeL2 = eph.code_on_L2;
    return out;
}

}  // namespace gps_sdr_sim
}  // namespace third_party
