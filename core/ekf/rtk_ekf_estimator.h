#pragma once

#include <gtest/gtest_prod.h>

#include <set>
#include <variant>

#include "common/logging/log_writer.h"
#include "common/satellite/satellite_cache.h"
#include "common/utctime.h"
#include "core/ekf/rtk_ekf.h"
#include "core/sat_manager.h"

namespace mc {
namespace ekf {

struct ImuMeas
{
    using Covariance = Eigen::DiagonalMatrix<double, 6>;
    using ZType = Input;
    ZType z;

    static ImuMeas Zero();
};

namespace detail {
template <class... Ts>
struct overloaded : Ts...
{
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

static constexpr int cacheId(int gnss_id, int sat_num)
{
    return (gnss_id << 8) | sat_num;
}

template <typename Ephemeris>
static constexpr int cacheId(int sat_num)
{
    if constexpr (std::is_same_v<Ephemeris, ephemeris::GPSEphemeris>)
    {
        return cacheId(GnssID::GPS, sat_num);
    }
    else if constexpr (std::is_same_v<Ephemeris, ephemeris::GalileoEphemeris>)
    {
        return cacheId(GnssID::Galileo, sat_num);
    }
    else if constexpr (std::is_same_v<Ephemeris, ephemeris::GlonassEphemeris>)
    {
        return cacheId(GnssID::Glonass, sat_num);
    }
    else
    {
        static_assert("Unknown Type");
    }
}

}  // namespace detail

class RtkEkfEstimator
{
 public:
    struct Meas
    {
        UTCTime t;
        std::variant<ImuMeas, pointPosMeas, gpsObsMeas, galObsMeas, gloObsMeas, fixAndHoldMeas> z;

        inline bool operator<(const Meas& other) const
        {
            if (t == other.t)
            {
                return z.index() < other.z.index();
            }
            else
            {
                return t < other.t;
            }
        }
    };
    using MeasIterator = typename std::multiset<Meas>::iterator;

    void init(const UTCTime& t0,
              const State& x0,
              const Covariance& P0,
              const pointPosMeas::Covariance& point_pos_cov,
              const gpsObsMeas::Covariance& gps_obs_cov,
              const galObsMeas::Covariance& gal_obs_cov,
              const gloObsMeas::Covariance& glo_obs_cov,
              const fixAndHoldMeas::Covariance& fix_and_hold_cov,
              const InputCovariance& imu_cov,
              const ProcessCovariance& process_cov,
              logging::Logger* log = nullptr);

    template <typename T>
    Error addMeasurement(const UTCTime& meas_time, const T& z)
    {
        if (meas_time < states_.front().x.t)
        {
            error("Unable to handle measurement with time {}, oldest state in buffer time {}",
                  fmt(meas_time, states_.front().x.t));
            return Error::create("cannot handle stale measurement");
        }

        if constexpr (std::is_same_v<T, gpsObsMeas>)
        {
            if (!hasCache(GnssID::GPS, z.sat_id))
            {
                warn("Unknown GPS Satellite {}", fmt(z.sat_id));
                return Error::create("Unknown satellite");
            }
        }
        else if constexpr (std::is_same_v<T, galObsMeas>)
        {
            if (!hasCache(GnssID::Galileo, z.sat_id))
            {
                warn("Unknown Galileo Satellite {}", fmt(z.sat_id));
                return Error::create("Unknown satellite");
            }
        }
        else if constexpr (std::is_same_v<T, gloObsMeas>)
        {
            if (!hasCache(GnssID::Glonass, z.sat_id))
            {
                warn("Unknown Glonass Satellite {}", fmt(z.sat_id));
                return Error::create("Unknown satellite");
            }
        }

        auto it = measurements_.insert(Meas{meas_time, z});

        if (meas_time < t())
        {
            // Out-of-order measurement.  We need to rewind to apply it.
            MeasIterator catchup_begin = measurements_.lower_bound(Meas{meas_time, {}});

            // rewind the state buffer
            while (t() > catchup_begin->t)
            {
                states_.pop_back();
            }
        }

        rollFilter(it, measurements_.end());

        dropOldMeasurements();

        return Error::none();
    }

    void dropOldMeasurements();

    bool hasCache(int gnss_id, int sat_num) const;

    const satellite::SatelliteCache& getCache(const UTCTime& t, int gnss_id, int sat_id) const;

    void handleImu(const UTCTime& meas_time, const ImuMeas& z);
    void handleImu(const UTCTime& meas_time, const Input& z);

    void handlePointPos(const UTCTime& meas_time, const pointPosMeas& z);
    void handleFixAndHold(const UTCTime& meas_time, const fixAndHoldMeas& z);
    void handleGpsObsMeas(const UTCTime& meas_time, const gpsObsMeas& z);
    void handleGalObsMeas(const UTCTime& meas_time, const galObsMeas& z);
    void handleGloObsMeas(const UTCTime& meas_time, const gloObsMeas& z);

    void handleMeas(const Meas& meas);

    const Meas& rollFilter(const MeasIterator& start, const MeasIterator& end);

    const Snapshot& snap() const { return states_.back(); }
    Snapshot& snap() { return states_.back(); }

    const UTCTime& t() const { return states_.back().x.t; }
    UTCTime& t() { return states_.back().x.t; }

    const State& x() const { return states_.back().x; }
    State& x() { return states_.back().x; }

    const Input& u() const { return states_.back().u; }
    Input& u() { return states_.back().u; }

    const Covariance& cov() const { return states_.back().cov; }
    Covariance& cov() { return states_.back().cov; }

    Vec3 p_ecef() const;

    std::multiset<Meas> measurements_;
    mutable std::unordered_map<int, satellite::SatelliteCache> sat_cache_;

    pointPosMeas::Covariance point_pos_cov_;
    gpsObsMeas::Covariance gps_obs_cov_;
    galObsMeas::Covariance gal_obs_cov_;
    gloObsMeas::Covariance glo_obs_cov_;
    fixAndHoldMeas::Covariance fix_and_hold_cov_;
    InputCovariance imu_cov_;
    ProcessCovariance process_cov_;

    CircularBuffer<Snapshot> states_;
    int state_idx_ = 0;

    Meas last_catchup_meas_;
    static constexpr double MAX_DELAY_AGE_S = 1.0;

    SatelliteManager sat_manager_;

    logging::Logger* log_ = nullptr;

    template <typename Ephemeris>
    void ephCb(const Ephemeris& eph)
    {
        const int cache_id = detail::cacheId<Ephemeris>(eph.sat);
        if (sat_cache_.find(cache_id) == sat_cache_.end())
        {
            sat_cache_.insert({cache_id, satellite::SatelliteCache()});
        }
        sat_manager_.ephCb(eph);
    }

    FRIEND_TEST(RtkEkfEstimator, OrderMeasurements);
};

}  // namespace ekf
}  // namespace mc
