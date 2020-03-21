#include "common/satellite/satellite_state.h"
#include "common/defs.h"
#include "common/math/rk4.h"
#include "common/print.h"

static constexpr double MAXDTOE = 7200;  // Maximum allowable age of ephemeris (s)

namespace mc {
namespace satellite {

Error eph2Sat(const UTCTime& t,
              const ephemeris::KeplerianEphemeris& eph,
              Out<SatelliteState> sat_state)
{
    using std::abs;
    using std::atan2;
    using std::cos;
    using std::sin;
    using std::sqrt;

    const double dt = (t - eph.toe).toSec();
    if (fabs(dt) > MAXDTOE)
    {
        error("Ephemeris dt = {}.  eph.toe = {}, t = {}", fmt(dt, eph.toe.str(), t.str()));
        return Error::create("Stale Ephemeris");
    }

    // https://www.ngs.noaa.gov/gps-toolbox/bc_velo/bc_velo.c
    const double A = eph.sqrta * eph.sqrta;
    const double n0 = sqrt(GM_EARTH / (A * A * A));
    const double mkdot = n0 + eph.delta_n;
    const double mk = eph.m0 + mkdot * dt;

    int i = 0;
    double ek_prev = std::numeric_limits<double>::max();
    double ek = mk;
    while (abs(ek - ek_prev) > 1e-13 && i < 30)
    {
        ek_prev = ek;
        ek -= (ek - eph.ecc * sin(ek) - mk) / (1.0 - eph.ecc * cos(ek));
        i++;
    }
    const double sek = sin(ek);
    const double cek = cos(ek);

    const double ekdot = mkdot / (1.0 - eph.ecc * cek);

    const double tak = atan2(sqrt(1.0 - eph.ecc * eph.ecc) * sek, cek - eph.ecc);
    const double takdot =
        sek * ekdot * (1.0 + eph.ecc * cos(tak)) / (sin(tak) * (1.0 - eph.ecc * cek));

    const double phik = tak + eph.w;
    const double sphik2 = sin(2.0 * phik);
    const double cphik2 = cos(2.0 * phik);
    const double corr_u = eph.cus * sphik2 + eph.cuc * cphik2;
    const double corr_r = eph.crs * sphik2 + eph.crc * cphik2;
    const double corr_i = eph.cis * sphik2 + eph.cic * cphik2;
    const double uk = phik + corr_u;
    const double rk = A * (1.0 - eph.ecc * cek) + corr_r;
    const double ik = eph.i0 + eph.idot * dt + corr_i;

    const double s2uk = sin(2.0 * uk);
    const double c2uk = cos(2.0 * uk);

    const double ukdot = takdot + 2.0 * (eph.cus * c2uk - eph.cuc * s2uk) * takdot;
    const double rkdot = A * eph.ecc * sek * mkdot / (1.0 - eph.ecc * cek) +
                         2.0 * (eph.crs * c2uk - eph.crc * s2uk) * takdot;
    const double ikdot = eph.idot + (eph.cis * c2uk - eph.cic * s2uk) * 2.0 * takdot;

    const double cuk = cos(uk);
    const double suk = sin(uk);

    const double xpk = rk * cuk;
    const double ypk = rk * suk;

    const double xpkdot = rkdot * cuk - ypk * ukdot;
    const double ypkdot = rkdot * suk + xpk * ukdot;

    const double omegak = eph.omega0 + (eph.omegadot - OMEGA_EARTH) * dt - OMEGA_EARTH * eph.toes;
    const double omegakdot = eph.omegadot - OMEGA_EARTH;

    const double cwk = cos(omegak);
    const double swk = sin(omegak);
    const double cik = cos(ik);
    const double sik = sin(ik);

    sat_state->pos.x() = xpk * cwk - ypk * swk * cik;
    sat_state->pos.y() = xpk * swk + ypk * cwk * cik;
    sat_state->pos.z() = ypk * sik;

    sat_state->vel.x() = (xpkdot - ypk * cik * omegakdot) * cwk -
                         (xpk * omegakdot + ypkdot * cik - ypk * sik * ikdot) * swk;
    sat_state->vel.y() = (xpkdot - ypk * cik * omegakdot) * swk +
                         (xpk * omegakdot + ypkdot * cik - ypk * sik * ikdot) * cwk;
    sat_state->vel.z() = ypkdot * sik + ypk * cik * ikdot;

    const double clk_dt = (t - eph.toc).toSec();
    double dts = eph.af0 + eph.af1 * dt + eph.af2 * clk_dt * clk_dt;

    // Correct for relativistic effects on the satellite clock
    dts -= 2.0 * sqrt(GM_EARTH * A) * eph.ecc * sek / (C_LIGHT * C_LIGHT);

    sat_state->clk(0) = dts * 1e9;                           // satellite clock bias (nsec)
    sat_state->clk(1) = (eph.af1 + eph.af2 * clk_dt) * 1e9;  // satellite drift rate (nsec/s)
    sat_state->t = t;

    return Error::none();
}

Error eph2Sat(const UTCTime& t,
              const ephemeris::GlonassEphemeris& eph,
              Out<SatelliteState> sat_state)
{
    double dt = (t - eph.toe).toSec();
    static constexpr double TSTEP = 60.0; /* integration step glonass ephemeris (s) */

    using std::abs;

    // Clock Biases
    sat_state->clk(0) = (-eph.taun + eph.gamn * dt) * 1e9;
    sat_state->clk(1) = (eph.gamn) * 1e9;

    Vec6 x;
    auto p = x.head<3>();
    auto v = x.tail<3>();
    p = eph.pos;
    v = eph.vel;

    double timestep = dt < 0.0 ? -TSTEP : TSTEP;
    while (abs(dt) > 1e-9)
    {
        if (abs(dt) < TSTEP)
        {
            timestep = dt;
        }
        std::function<Vec6(const Vec6&, const Vec3&)> f = &glonassOrbit;
        x = math::RK4(glonassOrbit, timestep, x, eph.acc);
        dt -= timestep;
    }

    sat_state->pos = x.head<3>();
    sat_state->vel = x.tail<3>();

    sat_state->t = t;

    return Error::none();
}

Vec6 glonassOrbit(const Vec6& x, const Vec3& acc)
{
    static constexpr double OMGE_GLO = 7.292115E-5; /* earth angular velocity (rad/s) ref [2] */
    static constexpr double RE_GLO = 6378136.0;     /* radius of earth (m)            ref [2] */
    static constexpr double J2_GLO = 1.0826257E-3;  /* 2nd zonal harmonic of geopot   ref [2] */
    static constexpr double MU_GLO = 3.9860044E14;  /* gravitational constant         ref [2] */
    auto p = x.head<3>();
    auto v = x.tail<3>();
    Vec6 xdot;
    auto pdot = xdot.head<3>();
    auto vdot = xdot.tail<3>();
    const double r2 = p.transpose() * p;
    const double r3 = r2 * std::sqrt(r2);
    const double r5 = r2 * r3;
    const double omg2 = OMGE_GLO * OMGE_GLO;
    const double re2 = RE_GLO * RE_GLO;

    double a = 1.5 * J2_GLO * MU_GLO * re2 / r5;  // 3/2*J2*mu*Ae^2/r^5
    double b = 5.0 * p.z() * p.z() / r2;
    double c = -MU_GLO / r3 - a * (1.0 - b);
    pdot = v;
    vdot.x() = (c + omg2) * p.x() + 2.0 * OMGE_GLO * v.y() + acc.x();
    vdot.y() = (c + omg2) * p.y() - 2.0 * OMGE_GLO * v.x() + acc.y();
    vdot.z() = (c - 2.0 * a) * p.z() + acc.z();
    return xdot;
}

}  // namespace satellite
}  // namespace mc
