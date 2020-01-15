#include "common/satellite/satellite_state.h"

static constexpr double GM_EARTH = 3.986005e14;         // Mass of the earth (kg)
static constexpr double OMEGA_EARTH = 7.2921151467e-5;  // Angular Velocity of the earth (rad/s)
static constexpr double C_LIGHT = 299792458.0;          // speed of light (m/s)

void eph2Sat(const UTCTime& t, const KeplerianEphemeris& eph, SatelliteState* sat_state)
{
    // // https://www.ngs.noaa.gov/gps-toolbox/bc_velo/bc_velo.c
    // const double A = eph.sqrta * eph.sqrta;
    // double n0 = std::sqrt(GM_EARTH / (A * A * A));
    // double mkdot = n0 + eph.delta_n;
    // double mk = eph.m0 + mkdot * dt;

    // int i = 0;
    // double ek_prev;
    // double ek = mk;
    // while (std::abs(ek - ek_prev) > 1e-13 && i < 30)
    // {
    //     ek_prev = ek;
    //     ek -= (ek - eph.ecc * std::sin(ek) - mk) / (1.0 - eph.ecc * std::cos(ek));
    //     i++;
    // }
    // double sek = std::sin(ek);
    // double cek = std::cos(ek);

    // double ekdot = mkdot / (1.0 - eph.ecc * cek);

    // double tak = std::atan2(std::sqrt(1.0 - eph.ecc * eph.ecc) * sek, cek - eph.ecc);
    // double takdot =
    //     sek * ekdot * (1.0 + eph.ecc * std::cos(tak)) / (std::sin(tak) * (1.0 - eph.ecc * cek));

    // double phik = tak + eph.omg;
    // double sphik2 = std::sin(2.0 * phik);
    // double cphik2 = std::cos(2.0 * phik);
    // double corr_u = eph.cus * sphik2 + eph.cuc * cphik2;
    // double corr_r = eph.crs * sphik2 + eph.crc * cphik2;
    // double corr_i = eph.cis * sphik2 + eph.cic * cphik2;
    // double uk = phik + corr_u;
    // double rk = A * (1.0 - eph.ecc * cek) + corr_r;
    // double ik = eph.i0 + eph.idot * dt + corr_i;

    // double s2uk = std::sin(2.0 * uk);
    // double c2uk = std::cos(2.0 * uk);

    // double ukdot = takdot + 2.0 * (eph.cus * c2uk - eph.cuc * s2uk) * takdot;
    // double rkdot = A * eph.ecc * sek * mkdot / (1.0 - eph.ecc * cek) +
    //                2.0 * (eph.crs * c2uk - eph.crc * s2uk) * takdot;
    // double ikdot = eph.idot + (eph.cis * c2uk - eph.cic * s2uk) * 2.0 * takdot;

    // double cuk = std::cos(uk);
    // double suk = std::sin(uk);

    // double xpk = rk * cuk;
    // double ypk = rk * suk;

    // double xpkdot = rkdot * cuk - ypk * ukdot;
    // double ypkdot = rkdot * suk + xpk * ukdot;

    // double omegak = eph.OMG0 + (eph.OMGd - OMEGA_EARTH) * dt - OMEGA_EARTH * eph.toes;
    // double omegakdot = eph.OMGd - OMEGA_EARTH;

    // double cwk = std::cos(omegak);
    // double swk = std::sin(omegak);
    // double cik = std::cos(ik);
    // double sik = std::sin(ik);

    // pos.x() = xpk * cwk - ypk * swk * cik;
    // pos.y() = xpk * swk + ypk * cwk * cik;
    // pos.z() = ypk * sik;

    // vel.x() = (xpkdot - ypk * cik * omegakdot) * cwk -
    //           (xpk * omegakdot + ypkdot * cik - ypk * sik * ikdot) * swk;
    // vel.y() = (xpkdot - ypk * cik * omegakdot) * swk +
    //           (xpk * omegakdot + ypkdot * cik - ypk * sik * ikdot) * cwk;
    // vel.z() = ypkdot * sik + ypk * cik * ikdot;

    // dt = (time - eph.toc).toSec();
    // double dts = eph.f0 + eph.f1 * dt + eph.f2 * dt * dt;

    // // Correct for relativistic effects on the satellite clock
    // dts -= 2.0 * std::sqrt(GM_EARTH * A) * eph.ecc * sek / (C_LIGHT * C_LIGHT);

    // clk(0) = dts;                   // satellite clock bias
    // clk(1) = eph.f1 + eph.f2 * dt;  // satellite drift rate
}
void eph2Sat(const UTCTime& t, const GlonassEphemeris& eph, SatelliteState* sat_state)
{
    assert(false);
}
