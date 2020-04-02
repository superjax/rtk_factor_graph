#include "common/logging/header.h"

#include <fstream>
#include <regex>

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/math/dquat.h"
#include "common/math/jet.h"
#include "common/math/quat.h"
#include "common/measurements/gnss_observation.h"
#include "common/measurements/imu.h"
#include "common/utctime.h"

namespace mc {
namespace logging {

std::string format(const UTCTime& time, const std::string& name, int pad)
{
    std::stringstream ss;
    ss << pad_format(pad, "{} {{\n", key(name));
    ss << logging::format(time.sec, "sec", pad + 2) << ",\n";
    ss << logging::format(time.nsec, "nsec", pad + 2) << "\n";
    ss << pad_format(pad, "}}");
    return ss.str();
}

std::string format(const ephemeris::GPSEphemeris& eph, const std::string& name, int pad)
{
    std::stringstream ss;
    ss << pad_format(pad, "{} {{\n", key(name));
    ss << format(eph.gnssID, "gnss_id", pad + 2) << ",\n";
    ss << format(eph.sat, "sat", pad + 2) << ",\n";
    ss << format(eph.toe, "toe", pad + 2) << ",\n";
    ss << format(eph.toc, "toc", pad + 2) + ",\n";
    ss << format(eph.tow, "tow", pad + 2) + ",\n";
    ss << format(eph.iodc, "iodc", pad + 2) + ",\n";
    ss << format(eph.iode, "iode", pad + 2) + ",\n";
    ss << format(eph.week, "week", pad + 2) + ",\n";
    ss << format(eph.toes, "toes", pad + 2) + ",\n";
    ss << format(eph.tocs, "tocs", pad + 2) + ",\n";
    ss << format(eph.af2, "af2", pad + 2) + ",\n";
    ss << format(eph.af1, "af1", pad + 2) + ",\n";
    ss << format(eph.af0, "af0", pad + 2) + ",\n";
    ss << format(eph.m0, "m0", pad + 2) + ",\n";
    ss << format(eph.delta_n, "delta_n", pad + 2) + ",\n";
    ss << format(eph.ecc, "ecc", pad + 2) + ",\n";
    ss << format(eph.sqrta, "sqrta", pad + 2) + ",\n";
    ss << format(eph.omega0, "omega0", pad + 2) + ",\n";
    ss << format(eph.i0, "i0", pad + 2) + ",\n";
    ss << format(eph.w, "w", pad + 2) + ",\n";
    ss << format(eph.omegadot, "omegadot", pad + 2) + ",\n";
    ss << format(eph.idot, "idot", pad + 2) + ",\n";
    ss << format(eph.cuc, "cuc", pad + 2) + ",\n";
    ss << format(eph.cus, "cus", pad + 2) + ",\n";
    ss << format(eph.crc, "crc", pad + 2) + ",\n";
    ss << format(eph.crs, "crs", pad + 2) + ",\n";
    ss << format(eph.cic, "cic", pad + 2) + ",\n";
    ss << format(eph.cis, "cis", pad + 2) + ",\n";
    ss << format(eph.health, "health", pad + 2) + ",\n";
    ss << format(eph.alert_flag, "alert_flag", pad + 2) + ",\n";
    ss << format(eph.anti_spoof, "anti_spoof", pad + 2) + ",\n";
    ss << format(eph.code_on_L2, "code_on_L2", pad + 2) + ",\n";
    ss << format(eph.ura, "ura", pad + 2) + ",\n";
    ss << format(eph.L2_P_data_flag, "L2_P_data_flag", pad + 2) + ",\n";
    ss << format(eph.fit_interval_flag, "fit_interval_flag", pad + 2) + ",\n";
    ss << format(eph.age_of_data_offset, "age_of_data_offset", pad + 2) + ",\n";
    ss << format(eph.tgd, "tgd", pad + 2) + ",\n";
    ss << format(eph.iode1, "iode1", pad + 2) + ",\n";
    ss << format(eph.iode2, "iode2", pad + 2) + ",\n";
    ss << format(eph.iode3, "iode3", pad + 2) + ",\n";
    ss << format(eph.collected_subframes, "collected_subframes", pad + 2) + "\n";
    ss << pad_format(pad, "}}");
    return ss.str();
}
std::string format(const ephemeris::GalileoEphemeris& eph, const std::string& name, int pad)
{
    std::stringstream ss;
    ss << pad_format(pad, "{} {{\n", key(name));
    // BaseEph Members
    ss << format(eph.gnssID, "gnss_id", pad + 2) << ",\n";
    ss << format(eph.sat, "sat", pad + 2) << ",\n";
    ss << format(eph.toe, "toe", pad + 2) << ",\n";

    // KeplerEphemeris members
    ss << format(eph.toc, "toc", pad + 2) + ",\n";
    ss << format(eph.tow, "tow", pad + 2) + ",\n";
    ss << format(eph.iodc, "iodc", pad + 2) + ",\n";
    ss << format(eph.iode, "iode", pad + 2) + ",\n";
    ss << format(eph.week, "week", pad + 2) + ",\n";
    ss << format(eph.toes, "toes", pad + 2) + ",\n";
    ss << format(eph.tocs, "tocs", pad + 2) + ",\n";
    ss << format(eph.af2, "af2", pad + 2) + ",\n";
    ss << format(eph.af1, "af1", pad + 2) + ",\n";
    ss << format(eph.af0, "af0", pad + 2) + ",\n";
    ss << format(eph.m0, "m0", pad + 2) + ",\n";
    ss << format(eph.delta_n, "delta_n", pad + 2) + ",\n";
    ss << format(eph.ecc, "ecc", pad + 2) + ",\n";
    ss << format(eph.sqrta, "sqrta", pad + 2) + ",\n";
    ss << format(eph.omega0, "omega0", pad + 2) + ",\n";
    ss << format(eph.i0, "i0", pad + 2) + ",\n";
    ss << format(eph.w, "w", pad + 2) + ",\n";
    ss << format(eph.omegadot, "omegadot", pad + 2) + ",\n";
    ss << format(eph.idot, "idot", pad + 2) + ",\n";
    ss << format(eph.cuc, "cuc", pad + 2) + ",\n";
    ss << format(eph.cus, "cus", pad + 2) + ",\n";
    ss << format(eph.crc, "crc", pad + 2) + ",\n";
    ss << format(eph.crs, "crs", pad + 2) + ",\n";
    ss << format(eph.cic, "cic", pad + 2) + ",\n";
    ss << format(eph.cis, "cis", pad + 2) + ",\n";
    // galileo members
    ss << format(eph.ai0, "ai0", pad + 2) + ",\n";
    ss << format(eph.ai1, "ai1", pad + 2) + ",\n";
    ss << format(eph.ai2, "ai2", pad + 2) + ",\n";
    ss << format(eph.bgd_e1_e5a, "bgd_e1_e5a", pad + 2) + ",\n";
    ss << format(eph.bgd_e1_e5b, "bgd_e1_e5b", pad + 2) + ",\n";
    ss << format(eph.e5b_hs, "e5b_hs", pad + 2) + ",\n";
    ss << format(eph.e1b_hs, "e1b_hs", pad + 2) + ",\n";
    ss << format(eph.e5b_dvs, "e5b_dvs", pad + 2) + ",\n";
    ss << format(eph.e1b_dvs, "e1b_dvs", pad + 2) + ",\n";
    ss << format(eph.collected_subframes, "collected_subframes", pad + 2) + ",\n";
    ss << format(eph.time_f, "time_f", pad + 2) + ",\n";
    ss << format(eph.iod, "iod", 4, pad + 2) + "\n";
    ss << pad_format(pad, "}}");
    return ss.str();
}

std::string format(const ephemeris::GlonassEphemeris& eph, const std::string& name, int pad)
{
    std::stringstream ss;
    ss << pad_format(pad, "{} {{\n", key(name));
    // BaseEph Members
    ss << format(eph.gnssID, "gnss_id", pad + 2) << ",\n";
    ss << format(eph.sat, "sat", pad + 2) << ",\n";
    ss << format(eph.toe, "toe", pad + 2) << ",\n";
    // Glonass members
    ss << format(eph.iode, "iode", pad + 2) << ",\n";
    ss << format(eph.slot, "slot", pad + 2) << ",\n";
    ss << format(eph.svh, "svh", pad + 2) << ",\n";
    ss << format(eph.sva, "sva", pad + 2) << ",\n";
    ss << format(eph.age, "age", pad + 2) << ",\n";
    ss << format(eph.tof, "tof", pad + 2) << ",\n";
    ss << format(eph.pos, "pos", pad + 2) << ",\n";
    ss << format(eph.vel, "vel", pad + 2) << ",\n";
    ss << format(eph.acc, "acc", pad + 2) << ",\n";
    ss << format(eph.taun, "taun", pad + 2) << ",\n";
    ss << format(eph.gamn, "gamn", pad + 2) << ",\n";
    ss << format(eph.dtaun, "dtaun", pad + 2) << ",\n";
    ss << format(eph.frame_flags_, "frame_flags", pad + 2) << ",\n";
    ss << format(eph.time_mark_, "time_mark", pad + 2) << ",\n";
    ss << format(eph.tk_h_, "tk_h", pad + 2) << ",\n";
    ss << format(eph.tk_m_, "tk_m", pad + 2) << ",\n";
    ss << format(eph.tk_s_, "tk_s", pad + 2) << ",\n";
    ss << format(eph.tb_, "tb", pad + 2) << ",\n";
    ss << format(eph.NT_, "NT", pad + 2) << ",\n";
    ss << format(eph.N4_, "N4", pad + 2) << ",\n";
    ss << format(eph.NA_, "NA", pad + 2) << "\n";
    ss << pad_format(pad, "}}");
    return ss.str();
}

Error getHeaderTime(const std::string& header_file, Out<UTCTime> t0)
{
    std::regex e(R"(\ntime:\s*\{\s*sec:\s*(\d*),\s*nsec:\s(\d*)\s*\})");
    std::ifstream t(header_file);
    std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    std::smatch match;
    while (std::regex_search(str, match, e))
    {
        if (match.size() < 3)
        {
            return Error::create("Unable to retrieve start time from header");
        }
        else
        {
            t0->sec = std::stol(match[1].str());
            t0->nsec = std::stol(match[2].str());
            return Error::none();
        }
    }
    return Error::create("Unable to retrieve start time from header");
}

std::string format(const meas::ImuSample& imu, const std::string& name, int pad)
{
    std::stringstream ss;
    ss << pad_format(pad, "{} {{\n", key(name));
    ss << format(imu.t, "t", pad + 2) << ",\n";
    ss << format(imu.accel, "accel", pad + 2) << ",\n";
    ss << format(imu.gyro, "gyro", pad + 2) << "\n";
    ss << pad_format(pad, "}}");
    return ss.str();
}

std::string format(const meas::GnssObservation& obs, const std::string& name, int pad)
{
    std::stringstream ss;
    ss << pad_format(pad, "{} {{\n", key(name));
    ss << format(obs.t, "t", pad + 2) << ",\n";
    ss << format(obs.gnss_id, "gnss_id", pad + 2) << ",\n";
    ss << format(obs.sat_num, "san_num", pad + 2) << ",\n";
    ss << format(obs.freq, "freq", pad + 2) << ",\n";
    ss << format(obs.pseudorange, "pseudorange", pad + 2) << ",\n";
    ss << format(obs.doppler, "doppler", pad + 2) << ",\n";
    ss << format(obs.carrier_phase, "carrier_phase", pad + 2) << "\n";
    ss << pad_format(pad, "}}");
    return ss.str();
}

std::string format(const math::Quat<double>& q, const std::string& name, int pad)
{
    std::stringstream ss;
    ss << pad_format(pad, "{} {{\n", key(name));
    ss << format(q.w(), "w", pad + 2) << ",\n";
    ss << format(q.x(), "x", pad + 2) << ",\n";
    ss << format(q.y(), "y", pad + 2) << ",\n";
    ss << format(q.z(), "z", pad + 2) << "\n";
    ss << pad_format(pad, "}}");
    return ss.str();
}

std::string format(const math::DQuat<double>& dq, const std::string& name, int pad)
{
    std::stringstream ss;
    ss << pad_format(pad, "{} {{\n", key(name));
    ss << format(dq.real(), "rot", pad + 2) << ",\n";
    ss << format(dq.translation(), "trans", pad + 2) << "\n";
    ss << pad_format(pad, "}}");
    return ss.str();
}

std::string format(const math::DQuat<double>::TangentVector& dx, const std::string& name, int pad)
{
    std::stringstream ss;
    ss << pad_format(pad, "{} {{\n", key(name));
    ss << format(dx.angular(), "angular", pad + 2) << ",\n";
    ss << format(dx.linear(), "linear", pad + 2) << "\n";
    ss << pad_format(pad, "}}");
    return ss.str();
}

std::string format(const math::Jet<double>& x, const std::string& name, int pad)
{
    std::stringstream ss;
    ss << pad_format(pad, "{} {{\n", key(name));
    ss << format(x.x, "x", pad + 2) << ",\n";
    ss << format(x.dx, "dx", pad + 2) << "\n";
    ss << pad_format(pad, "}}");
    return ss.str();
}

}  // namespace logging
}  // namespace mc
