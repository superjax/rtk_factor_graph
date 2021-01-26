#include <fstream>
#include <regex>

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/logging/log_format.h"
#include "common/math/dquat.h"
#include "common/math/jet.h"
#include "common/math/quat.h"
#include "common/math/two_jet.h"
#include "common/measurements/gnss_observation.h"
#include "common/measurements/imu.h"
#include "common/utctime.h"

namespace mc {
namespace logging {

YAML::Node format(const UTCTime& time)
{
    YAML::Node out;
    out["sec"] = format(time.sec);
    out["nsec"] = format(time.nsec);
    return out;
}

YAML::Node format(const ephemeris::GPSEphemeris& eph)
{
    YAML::Node node;
    node["gnss_id"] = format(eph.gnssID);
    node["sat"] = format(eph.sat);
    node["toe"] = format(eph.toe);
    node["toc"] = format(eph.toc);
    node["tow"] = format(eph.tow);
    node["iodc"] = format(eph.iodc);
    node["iode"] = format(eph.iode);
    node["week"] = format(eph.week);
    node["toes"] = format(eph.toes);
    node["tocs"] = format(eph.tocs);
    node["af2"] = format(eph.af2);
    node["af1"] = format(eph.af1);
    node["af0"] = format(eph.af0);
    node["m0"] = format(eph.m0);
    node["delta_n"] = format(eph.delta_n);
    node["ecc"] = format(eph.ecc);
    node["sqrta"] = format(eph.sqrta);
    node["omega0"] = format(eph.omega0);
    node["i0"] = format(eph.i0);
    node["w"] = format(eph.w);
    node["omegadot"] = format(eph.omegadot);
    node["idot"] = format(eph.idot);
    node["cuc"] = format(eph.cuc);
    node["cus"] = format(eph.cus);
    node["crc"] = format(eph.crc);
    node["crs"] = format(eph.crs);
    node["cic"] = format(eph.cic);
    node["cis"] = format(eph.cis);
    node["health"] = format(eph.health);
    node["alert_flag"] = format(eph.alert_flag);
    node["anti_spoof"] = format(eph.anti_spoof);
    node["code_on_L2"] = format(eph.code_on_L2);
    node["ura"] = format(eph.ura);
    node["L2_P_data_flag"] = format(eph.L2_P_data_flag);
    node["fit_interval_flag"] = format(eph.fit_interval_flag);
    node["age_of_data_offset"] = format(eph.age_of_data_offset);
    node["tgd"] = format(eph.tgd);
    node["iode1"] = format(eph.iode1);
    node["iode2"] = format(eph.iode2);
    node["iode3"] = format(eph.iode3);
    node["collected_subframes"] = format(eph.collected_subframes);
    return node;
}
YAML::Node format(const ephemeris::GalileoEphemeris& eph)
{
    YAML::Node node;
    // BaseEph Members
    node["gnss_id"] = format(eph.gnssID);
    node["sat"] = format(eph.sat);
    node["toe"] = format(eph.toe);

    // KeplerEphemeris members
    node["toc"] = format(eph.toc);
    node["tow"] = format(eph.tow);
    node["iodc"] = format(eph.iodc);
    node["iode"] = format(eph.iode);
    node["week"] = format(eph.week);
    node["toes"] = format(eph.toes);
    node["tocs"] = format(eph.tocs);
    node["af2"] = format(eph.af2);
    node["af1"] = format(eph.af1);
    node["af0"] = format(eph.af0);
    node["m0"] = format(eph.m0);
    node["delta_n"] = format(eph.delta_n);
    node["ecc"] = format(eph.ecc);
    node["sqrta"] = format(eph.sqrta);
    node["omega0"] = format(eph.omega0);
    node["i0"] = format(eph.i0);
    node["w"] = format(eph.w);
    node["omegadot"] = format(eph.omegadot);
    node["idot"] = format(eph.idot);
    node["cuc"] = format(eph.cuc);
    node["cus"] = format(eph.cus);
    node["crc"] = format(eph.crc);
    node["crs"] = format(eph.crs);
    node["cic"] = format(eph.cic);
    node["cis"] = format(eph.cis);
    // galileo members
    node["ai0"] = format(eph.ai0);
    node["ai1"] = format(eph.ai1);
    node["ai2"] = format(eph.ai2);
    node["bgd_e1_e5a"] = format(eph.bgd_e1_e5a);
    node["bgd_e1_e5b"] = format(eph.bgd_e1_e5b);
    node["e5b_hs"] = format(eph.e5b_hs);
    node["e1b_hs"] = format(eph.e1b_hs);
    node["e5b_dvs"] = format(eph.e5b_dvs);
    node["e1b_dvs"] = format(eph.e1b_dvs);
    node["collected_subframes"] = format(eph.collected_subframes);
    node["time_f"] = format(eph.time_f);
    node["iod"] = format(eph.iod, 4);
    return node;
}

YAML::Node format(const ephemeris::GlonassEphemeris& eph)
{
    YAML::Node node;
    // BaseEph Members
    node["gnss_id"] = format(eph.gnssID);
    node["sat"] = format(eph.sat);
    node["toe"] = format(eph.toe);
    // Glonass members
    node["iode"] = format(eph.iode);
    node["slot"] = format(eph.slot);
    node["svh"] = format(eph.svh);
    node["sva"] = format(eph.sva);
    node["age"] = format(eph.age);
    node["tof"] = format(eph.tof);
    node["pos"] = format(eph.pos);
    node["vel"] = format(eph.vel);
    node["acc"] = format(eph.acc);
    node["taun"] = format(eph.taun);
    node["gamn"] = format(eph.gamn);
    node["dtaun"] = format(eph.dtaun);
    node["frame_flags"] = format(eph.frame_flags_);
    node["time_mark"] = format(eph.time_mark_);
    node["tk_h"] = format(eph.tk_h_);
    node["tk_m"] = format(eph.tk_m_);
    node["tk_s"] = format(eph.tk_s_);
    node["tb"] = format(eph.tb_);
    node["NT"] = format(eph.NT_);
    node["N4"] = format(eph.N4_);
    node["NA"] = format(eph.NA_);
    return node;
}

YAML::Node format(const meas::ImuSample& imu)
{
    YAML::Node node;
    node["t"] = format(imu.t);
    node["accel"] = format(imu.accel);
    node["gyro"] = format(imu.gyro);
    return node;
}

YAML::Node format(const meas::GnssObservation& obs)
{
    YAML::Node node;
    node["t"] = format(obs.t);
    node["gnss_id"] = format(obs.gnss_id);
    node["sat_num"] = format(obs.sat_num);
    node["freq"] = format(obs.freq);
    node["pseudorange"] = format(obs.pseudorange);
    node["doppler"] = format(obs.doppler);
    node["carrier_phase"] = format(obs.carrier_phase);
    return node;
}

YAML::Node format(const math::Quat<double>& q)
{
    YAML::Node node;
    node["w"] = format(q.w());
    node["x"] = format(q.x());
    node["y"] = format(q.y());
    node["z"] = format(q.z());
    return node;
}

YAML::Node format(const math::DQuat<double>& dq)
{
    YAML::Node node;
    node["rot"] = format(dq.real());
    node["trans"] = format(dq.translation());
    return node;
}

YAML::Node format(const math::DQuat<double>::TangentVector& dx)
{
    YAML::Node node;
    node["angular"] = format(dx.angular());
    node["linear"] = format(dx.linear());
    return node;
}

YAML::Node format(const math::Jet<double>& x)
{
    YAML::Node node;
    node["x"] = format(x.x);
    node["dx"] = format(x.dx);
    return node;
}
YAML::Node format(const math::TwoJet<double>& x)
{
    YAML::Node node;
    node["x"] = format(x.x);
    node["dx"] = format(x.dx);
    node["d2x"] = format(x.d2x);
    return node;
}

}  // namespace logging
}  // namespace mc
