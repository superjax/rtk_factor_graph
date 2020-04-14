#pragma once

#include <map>
#include <random>

#include "common/defs.h"
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
        std::map<int, std::string> ephemeris_files_;
        double update_rate_hz_;
        double pseudorange_stdev_;
        double carrier_phase_stdev_;
        double doppler_stdev_;
        double multipath_probability_;
        double loss_of_lock_probability_;
        double clock_walk_stdev_;
        double clock_init_stdev_;
        math::DQuat<double> T_e2n;
        Vec3 p_b2g;
    };

    GnssSim(const Options& options, const UTCTime& t0);

    satellite::SatelliteBase* get_sat(int gnss_id, int sat_id);

    bool sample(const UTCTime& t,
                const math::TwoJet<double>& x,
                Out<std::vector<meas::GnssObservation>> obs);

    void range(const UTCTime& t,
               satellite::SatelliteBase* sat,
               const Vec3& p_ecef,
               const Vec3& vel_ecef,
               Out<double> range,
               Out<double> range_rate);

    UTCTime prev_t_;
    Options options_;
    Vec2 clock_bias_;
    std::vector<int> carrier_phase_offsets_;
    std::vector<satellite::SatelliteBase*> satellites_;

    std::normal_distribution<double> normal_;
    std::default_random_engine gen_;
};

}  // namespace sensors
}  // namespace sim
}  // namespace mc
