#pragma once

#include <experimental/filesystem>
#include <fstream>
#include <string>

#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/error.h"
#include "common/logging/logger.h"
#include "common/out.h"
#include "common/print.h"
#include "common/utctime.h"

namespace mc {
namespace logging {

class LogReader
{
 public:
    LogReader(const std::string& filename) { open(filename); }
    LogReader() {}
    ~LogReader()
    {
        if (file_.is_open())
        {
            file_.close();
        }
    }

    Error open(const std::string& filename)
    {
        namespace fs = std::experimental::filesystem;
        if (!fs::exists(filename))
        {
            error("Unable to find log file {} for reading", fmt(filename));
            return Error::create("Unable to find file");
        }
        file_.open(filename);
        if (!file_.is_open())
        {
            error("Unable to open log file {}", fmt(filename));
            return Error::create("Unable to open file");
        }
        return Error::none();
    }

    template <typename... T>
    void read(T&... x)
    {
        int dummy[sizeof...(x)] = {(deserialize(x), 1)...};
        (void)dummy;
    }

    bool done() { return file_.eof(); }

 private:
    template <typename T>
    void deserialize(T& x)
    {
        if constexpr (detail::has_data<T>::value)
        {
            file_.read(reinterpret_cast<char*>(x.data()), sizeof(decltype(*x.data())) * x.size());
        }
        else
        {
            file_.read(reinterpret_cast<char*>(&x), sizeof(T));
        }
    }

    void deserialize(UTCTime& t) { read(t.sec, t.nsec); }

    void deserialize(ephemeris::GPSEphemeris& eph)
    {
        read(eph.gnssID, eph.sat, eph.toe, eph.toc, eph.tow, eph.iodc, eph.iode, eph.week, eph.toes,
             eph.tocs, eph.af2, eph.af1, eph.af0, eph.m0, eph.delta_n, eph.ecc, eph.sqrta,
             eph.omega0, eph.i0, eph.w, eph.omegadot, eph.idot, eph.cuc, eph.cus, eph.crc, eph.crs,
             eph.cic, eph.cis, eph.health, eph.alert_flag, eph.anti_spoof, eph.code_on_L2, eph.ura,
             eph.L2_P_data_flag, eph.fit_interval_flag, eph.age_of_data_offset, eph.tgd);
    }

    void deserialize(ephemeris::GlonassEphemeris& eph)
    {
        read(eph.gnssID, eph.sat, eph.toe, eph.iode, eph.slot, eph.svh, eph.sva, eph.age, eph.toe,
             eph.tof, eph.pos, eph.vel, eph.acc, eph.taun, eph.gamn, eph.dtaun);
    }

    void deserialize(ephemeris::GalileoEphemeris& eph)
    {
        read(eph.gnssID, eph.sat, eph.toe, eph.toc, eph.tow, eph.iodc, eph.iode, eph.week, eph.toes,
             eph.tocs, eph.af2, eph.af1, eph.af0, eph.m0, eph.delta_n, eph.ecc, eph.sqrta,
             eph.omega0, eph.i0, eph.w, eph.omegadot, eph.idot, eph.cuc, eph.cus, eph.crc, eph.crs,
             eph.cic, eph.cis, eph.ai0, eph.ai1, eph.ai2, eph.bgd_e1_e5a, eph.bgd_e1_e5b,
             eph.e5b_hs, eph.e1b_hs, eph.e5b_dvs, eph.e1b_dvs);
    }

    std::string filename_ = "";
    std::ifstream file_;
};

}  // namespace logging
}  // namespace mc
