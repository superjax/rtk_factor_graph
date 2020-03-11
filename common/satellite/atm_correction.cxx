#include "common/satellite/atm_correction.h"
#include "common/defs.h"
#include "common/error.h"
#include "common/satellite/azel.h"
#include "utils/wgs84.h"

namespace mc {
namespace satellite {

Error computeAtmCorrection(const UTCTime& t,
                           const Vec3& rec_pos_ecef,
                           const SatelliteState& sat_state,
                           Out<AtmosphericCorrection> atm)
{
    const AzimuthElevation azel = getAzEl(rec_pos_ecef, sat_state);
    const Vec3 rec_pos_lla = utils::WGS84::ecef2lla(rec_pos_ecef);
    atm->ionospheric_delay_m = computeIonosphericDelay(t, rec_pos_lla, azel);
    atm->tropospheric_delay_m = computeTroposphericDelay(t, rec_pos_lla, azel);
    return Error::none();
}

double computeTroposphericDelay(const UTCTime& t,
                                const Vec3& rec_pos_lla,
                                const AzimuthElevation& azel)
{
    using std::cos;
    using std::exp;
    using std::max;
    using std::pow;

    // Saastamoninen Troposphere Model
    const double temp0 = 15.0;  // temparature at sea level
    const double humi = 0.7;    // relative humidity

    if (rec_pos_lla.z() < -100.0 || 1E4 < rec_pos_lla.z() || azel.el <= 0)
        return 0.0;

    const double hgt = max(rec_pos_lla.z(), 0.0);

    // standard atmosphere
    const double pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
    const double temp = temp0 - 6.5E-3 * hgt + 273.16;
    const double e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45));

    // Saastamoninen model
    const double z = M_PI / 2.0 - azel.el;
    const double trph = 0.0022768 * pres /
                        (1.0 - 0.00266 * cos(2.0 * rec_pos_lla.x()) - 0.00028 * hgt / 1E3) / cos(z);
    const double trpw = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);

    return trph + trpw;
}

double computeIonosphericDelay(const UTCTime& utc_t, const Vec3& lla, const AzimuthElevation& azel)
{
    using std::cos;
    using std::floor;
    using std::max;
    using std::min;
    using std::sin;

    // Klobuchar Algorithm:
    // https://gssc.esa.int/navipedia/index.php/Klobuchar_Ionospheric_Model
    const double ion[] = {// as of 2004/1/1
                          0.1118E-07, -0.7451E-08, -0.5961E-07, 0.1192E-06,
                          0.1167E+06, -0.2294E+06, -0.1311E+06, 0.1049E+07};

    // Earth-Centered Angle (Elevation in Semicircles)
    const double psi = 0.0137 / (azel.el / M_PI + 0.11) - 0.022;

    // latitude of the ionosphere pierce point (IPP)
    const double phi_I = sat(lla(0) / M_PI + psi * cos(azel.az), -0.416, 0.416);

    // longitude of IPP
    const double lambda_I = lla(1) / M_PI + (psi * sin(azel.az) / cos(phi_I * M_PI));

    // geomagnetic latitude of the IPP
    const double phi_m = phi_I + 0.064 * cos((lambda_I - 1.617) * M_PI);

    // local time at hte IPP
    double t = 43200 * lambda_I + utc_t.GpsTow() + UTCTime::LEAP_SECONDS;
    t -= floor(t / 86400.0) * 86400.0;

    // Amplitude of Ionospheric Delay
    const double Amp = max(ion[0] + phi_m * (ion[1] + phi_m * (ion[2] + phi_m * ion[3])), 0.0);

    // Period of Ionospheric Delay
    const double Per = max(ion[4] + phi_m * (ion[5] + phi_m * (ion[6] + phi_m * ion[7])), 72'000.0);

    // Phase of Ionospheric Delay
    const double X_I = 2.0 * M_PI * (t - 50400.0) / Per;

    // Slant Factor
    const double F = 1.0 + 16.0 * pow((0.53 - azel.el / M_PI), 3.0);

    // Compute Ionospheric Time Delay (meters)
    if (std::abs(X_I) <= 1.57)
    {
        const double X2 = X_I * X_I;
        const double X4 = X2 * X2;
        return C_LIGHT * F * (5e-9 + Amp * (1.0 - X2 / 2.0 + X4 / 24.0));
    }
    else
        return C_LIGHT * F * 5e-9;
}

}  // namespace satellite
}  // namespace mc
