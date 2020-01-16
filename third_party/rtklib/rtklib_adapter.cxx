#include "third_party/rtklib/rtklib_adapter.h"

namespace rtklib
{
eph_t toRtklib(const KeplerianEphemeris& eph)
{
    rtklib::eph_t out;
    out.sat = eph.sat;  // Only works for GPS satellites
    out.A = eph.sqrta * eph.sqrta;
    out.toe.time = eph.toe.sec;
    out.toe.sec = eph.toe.nsec * 1e-9;
    out.toc.time = eph.toc.sec;
    out.toc.sec = eph.toc.nsec * 1e-9;
    out.toes = eph.toes;
    out.deln = eph.delta_n;
    out.M0 = eph.m0;
    out.e = eph.ecc;
    out.omg = eph.w;
    out.cus = eph.cus;
    out.cuc = eph.cuc;
    out.crs = eph.crs;
    out.crc = eph.crc;
    out.cis = eph.cis;
    out.cic = eph.cic;
    out.idot = eph.idot;
    out.i0 = eph.i0;
    out.OMG0 = eph.omega0;
    out.OMGd = eph.omegadot;
    out.f0 = eph.af0;
    out.f1 = eph.af1;
    out.f2 = eph.af2;
    return out;
}

gtime_t toRtklib(const UTCTime& t)
{
    gtime_t out;
    out.time = t.sec;
    out.sec = t.nsec * 1e-9;
    return out;
}
}  // namespace rtklib
