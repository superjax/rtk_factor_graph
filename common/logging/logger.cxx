#include "common/logging/logger.h"

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/math/dquat.h"
#include "common/math/jet.h"
#include "common/math/quat.h"
#include "common/math/two_jet.h"
#include "common/measurements/gnss_observation.h"
#include "common/measurements/imu.h"
#include "common/utctime.h"

namespace mc {
namespace logging {

void Logger::serialize(const UTCTime& t)
{
    log(t.sec, t.nsec);
}

void Logger::serialize(const ephemeris::GPSEphemeris& eph)
{
    log(eph.gnssID, eph.sat, eph.toe, eph.toc, eph.tow, eph.iodc, eph.iode, eph.week, eph.toes,
        eph.tocs, eph.af2, eph.af1, eph.af0, eph.m0, eph.delta_n, eph.ecc, eph.sqrta, eph.omega0,
        eph.i0, eph.w, eph.omegadot, eph.idot, eph.cuc, eph.cus, eph.crc, eph.crs, eph.cic, eph.cis,
        eph.health, eph.alert_flag, eph.anti_spoof, eph.code_on_L2, eph.ura, eph.L2_P_data_flag,
        eph.fit_interval_flag, eph.age_of_data_offset, eph.tgd);
}

void Logger::serialize(const ephemeris::GlonassEphemeris& eph)
{
    log(eph.gnssID, eph.sat, eph.toe, eph.iode, eph.slot, eph.svh, eph.sva, eph.age, eph.toe,
        eph.tof, eph.pos, eph.vel, eph.acc, eph.taun, eph.gamn, eph.dtaun);
}

void Logger::serialize(const ephemeris::GalileoEphemeris& eph)
{
    log(eph.gnssID, eph.sat, eph.toe, eph.toc, eph.tow, eph.iodc, eph.iode, eph.week, eph.toes,
        eph.tocs, eph.af2, eph.af1, eph.af0, eph.m0, eph.delta_n, eph.ecc, eph.sqrta, eph.omega0,
        eph.i0, eph.w, eph.omegadot, eph.idot, eph.cuc, eph.cus, eph.crc, eph.crs, eph.cic, eph.cis,
        eph.ai0, eph.ai1, eph.ai2, eph.bgd_e1_e5a, eph.bgd_e1_e5b, eph.e5b_hs, eph.e1b_hs,
        eph.e5b_dvs, eph.e1b_dvs);
}

void Logger::serialize(const meas::ImuSample& imu)
{
    log(imu.t, imu.accel, imu.gyro);
}

void Logger::serialize(const meas::GnssObservation& obs)
{
    log(obs.t, obs.gnss_id, obs.sat_num, obs.freq, obs.pseudorange, obs.doppler, obs.carrier_phase);
}

void Logger::serialize(const math::Quat<double>& q)
{
    log(q.arr_);
}

void Logger::serialize(const math::DQuat<double>& dq)
{
    log(dq.real(), dq.translation());
}

void Logger::serialize(const math::Jet<double>& x)
{
    log(x.x, x.dx);
}

void Logger::serialize(const math::TwoJet<double>& x)
{
    log(x.x, x.dx, x.d2x);
}

}  // namespace logging
}  // namespace mc
