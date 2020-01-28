#include "third_party/rtklib/rtklib_adapter.h"

namespace third_party {
namespace rtklib {

eph_t toRtklib(const mc::ephemeris::KeplerianEphemeris& eph)
{
    eph_t out;
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

gtime_t toRtklib(const mc::UTCTime& t)
{
    gtime_t out;
    out.time = t.sec;
    out.sec = t.nsec * 1e-9;
    return out;
}

geph_t toRtklib(const mc::ephemeris::GlonassEphemeris& eph)
{
    geph_t out;
    out.sat = eph.sat;
    out.iode = eph.iode;
    out.frq = eph.slot;
    out.svh = eph.svh;
    out.sva = eph.sva;
    out.age = eph.age;
    out.toe = toRtklib(eph.toe);
    out.tof = toRtklib(eph.tof);
    out.pos[0] = eph.pos[0];
    out.pos[1] = eph.pos[1];
    out.pos[2] = eph.pos[2];
    out.vel[0] = eph.vel[0];
    out.vel[1] = eph.vel[1];
    out.vel[2] = eph.vel[2];
    out.acc[0] = eph.acc[0];
    out.acc[1] = eph.acc[1];
    out.acc[2] = eph.acc[2];
    out.taun = eph.taun;
    out.gamn = eph.gamn;
    out.dtaun = eph.dtaun;
    return out;
}

}  // namespace rtklib
}  // namespace third_party
