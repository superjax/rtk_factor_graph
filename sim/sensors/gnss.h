#pragma once

#include <map>
#include <random>

#include "common/defs.h"
#include "common/ephemeris/galileo.h"
#include "common/ephemeris/glonass.h"
#include "common/ephemeris/gps.h"
#include "common/logging/log_reader.h"
#include "common/math/two_jet.h"
#include "common/matrix_defs.h"
#include "common/measurements/gnss_observation.h"
#include "common/out.h"
#include "common/satellite/satellite.h"
#include "common/utctime.h"

namespace mc {
namespace sim {
namespace sensors {

class GnssSim
{
 public:
    struct Options
    {
        std::string ephemeris_log;
        double update_rate_hz_ = 5;
        double pseudorange_stdev_ = 3.0;
        double carrier_phase_stdev_ = 0.01;
        double doppler_stdev_ = 1.0;
        double multipath_probability_ = 0.0;
        double loss_of_lock_probability_ = 0.0;
        double clock_walk_stdev_ = 1e-9;
        double clock_init_stdev_ = 1e-10;
        math::DQuat<double> T_e2n;
        Vec3 p_b2g;
    };

    GnssSim(const Options& options, const UTCTime& t0);

    bool sample(const UTCTime& t,
                const math::TwoJet<double>& x,
                Out<std::vector<meas::GnssObservation>> obs);

    bool sampleGpsEph(const UTCTime& t, Out<ephemeris::GPSEphemeris> obs);
    bool sampleGloEph(const UTCTime& t, Out<ephemeris::GlonassEphemeris> obs);
    bool sampleGalEph(const UTCTime& t, Out<ephemeris::GalileoEphemeris> obs);

    void range(const UTCTime& t,
               const satellite::SatelliteBase* sat,
               const Vec3& p_ecef,
               const Vec3& vel_ecef,
               Out<double> range,
               Out<double> range_rate);

    void load();

    math::DQuat<double> T_e2g_;
    std::string log_path_;
    UTCTime prev_t_;
    Options options_;
    Vec2 clock_bias_;
    std::vector<int> carrier_phase_offsets_;
    std::vector<const satellite::SatelliteBase*> satellites_;
    std::map<int, satellite::Satellite<ephemeris::GPSEphemeris>> gps_;
    std::map<int, satellite::Satellite<ephemeris::GlonassEphemeris>> glo_;
    std::map<int, satellite::Satellite<ephemeris::GalileoEphemeris>> gal_;

    std::vector<std::tuple<UTCTime, ephemeris::GPSEphemeris>> gps_ephemerides_;
    std::vector<std::tuple<UTCTime, ephemeris::GlonassEphemeris>> glo_ephemerides_;
    std::vector<std::tuple<UTCTime, ephemeris::GalileoEphemeris>> gal_ephemerides_;

    std::normal_distribution<double> normal_;
    std::default_random_engine gen_;
};

}  // namespace sensors
}  // namespace sim
}  // namespace mc
