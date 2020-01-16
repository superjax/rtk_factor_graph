#include "common/satellite/satellite_state.h"
#include "common/print.h"

static constexpr double GM_EARTH = 3.986005e14;         // Mass of the earth (kg)
static constexpr double OMEGA_EARTH = 7.2921151467e-5;  // Angular Velocity of the earth (rad/s)
static constexpr double C_LIGHT = 299792458.0;          // speed of light (m/s)
static constexpr double MAXDTOE = 7200;                 // Maximum allowable age of ephemeris (s)

Error eph2Sat(const UTCTime& t, const KeplerianEphemeris& eph, SatelliteState* sat_state)
{
    using std::abs;
    using std::atan2;
    using std::cos;
    using std::sin;
    using std::sqrt;

    const double dt = (t - eph.toe).toSec();
    if (fabs(dt) > MAXDTOE)
    {
        err("Ephemeris dt = %f.  eph.toe = %s, t = %s", dt, eph.toe.str().c_str(), t.str().c_str());
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

    sat_state->clk(0) = dts;                         // satellite clock bias
    sat_state->clk(1) = eph.af1 + eph.af2 * clk_dt;  // satellite drift rate

    return Error::none();
}

// void eph2Sat(const UTCTime& t, const GlonassEphemeris& eph, SatelliteState* sat_state)
// {
//     assert(false);
// }
