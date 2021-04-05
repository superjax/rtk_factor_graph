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
};

namespace detail {
template <class... Ts>
struct overloaded : Ts...
{
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;
}  // namespace detail

template <typename EkfType>
class EkfEstimator
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
              const Vec<60>& P0,
              const Vec6& point_pos_cov,
              const Vec2& gps_obs_cov,
              const Vec2& gal_obs_cov,
              const Vec2& glo_obs_cov,
              const double& fix_and_hold_cov,
              const Input& imu_cov,
              const Vec<60>& process_cov,
              logging::Logger* log = nullptr)
    {
        point_pos_cov_ = point_pos_cov.asDiagonal();
        gps_obs_cov_ = gps_obs_cov.asDiagonal();
        gal_obs_cov_ = gal_obs_cov.asDiagonal();
        glo_obs_cov_ = glo_obs_cov.asDiagonal();
        fix_and_hold_cov_ = Vec30::Constant(fix_and_hold_cov).asDiagonal();
        imu_cov_ = imu_cov.asDiagonal();
        process_cov_ = process_cov.asDiagonal();

        log_ = log;

        Input u0;
        u0.accel = -GRAVITY;
        u0.gyro.setZero();

        states_.emplace_back();
        x() = x0;
        t() = t0;
        cov() = P0.asDiagonal();
        u() = u0;
    }

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

    void dropOldMeasurements()
    {
        check(!measurements_.empty(), "cannot process empty buffer");

        const UTCTime t_start = measurements_.crbegin()->t - MAX_DELAY_AGE_S;
        measurements_.erase(measurements_.cbegin(), measurements_.lower_bound(Meas{t_start, {}}));
        while (states_.front().x.t < t_start)
        {
            states_.pop_front();
        }
    }

    mutable std::unordered_map<int, satellite::SatelliteCache> sat_cache_;

    static constexpr int cacheId(int gnss_id, int sat_num) { return (gnss_id << 8) | sat_num; }

    inline bool hasCache(int gnss_id, int sat_num) const
    {
        return sat_cache_.find(cacheId(gnss_id, sat_num)) != sat_cache_.end();
    }

    const satellite::SatelliteCache& getCache(const UTCTime& t, int gnss_id, int sat_id) const
    {
        auto& cache = sat_cache_.at(cacheId(gnss_id, sat_id));
        const satellite::SatelliteBase* sat;
        const Error res = sat_manager_.getSat(gnss_id, sat_id, make_out(sat));
        check(res.ok(), "unable to retrieve satellite");

        cache.update(t, ekf_.p_e_g2e(x()), *sat);
        return cache;
    }

    void handleImu(const UTCTime& meas_time, const ImuMeas& z) { handleImu(meas_time, z.z); }
    void handleImu(const UTCTime& meas_time, const Input& z)
    {
        Snapshot& snap = states_.back();
        states_.emplace_back();
        Snapshot& new_snap = states_.back();
        if (meas_time > t())
        {
            ekf_.predict(snap, meas_time, z, process_cov_, imu_cov_, make_out(new_snap));
        }
    }

    void handlePointPos(const UTCTime& meas_time, const pointPosMeas& z)
    {
        if (meas_time > t().quantized())
        {
            handleImu(meas_time, u());
        }
        ekf_.template update<pointPosMeas>(make_out(snap()), z.z, point_pos_cov_, u());
    }

    void handleFixAndHold(const UTCTime& meas_time, const fixAndHoldMeas& z)
    {
        if (meas_time > t().quantized())
        {
            handleImu(meas_time, u());
        }
        ekf_.template update<fixAndHoldMeas>(make_out(snap()), z.z, fix_and_hold_cov_);
    }

    void handleGpsObsMeas(const UTCTime& meas_time, const gpsObsMeas& z)
    {
        if (meas_time > t().quantized())
        {
            handleImu(meas_time, u());
        }
        const auto& cache = getCache(meas_time, GnssID::GPS, z.sat_id);
        ekf_.template update<gpsObsMeas>(make_out(snap()), z.z, gps_obs_cov_, u(), cache);
    }

    void handleGalObsMeas(const UTCTime& meas_time, const galObsMeas& z)
    {
        if (meas_time > t().quantized())
        {
            handleImu(meas_time, u());
        }
        const auto& cache = getCache(meas_time, GnssID::Galileo, z.sat_id);
        ekf_.template update<galObsMeas>(make_out(snap()), z.z, gps_obs_cov_, u(), cache);
    }

    void handleGloObsMeas(const UTCTime& meas_time, const gloObsMeas& z)
    {
        if (meas_time > t().quantized())
        {
            handleImu(meas_time, u());
        }
        const auto& cache = getCache(meas_time, GnssID::Glonass, z.sat_id);
        ekf_.template update<gloObsMeas>(make_out(snap()), z.z, gps_obs_cov_, u(), cache);
    }

    void handleMeas(const Meas& meas)
    {
        std::visit(detail::overloaded{
                       [&](const ImuMeas& z) { handleImu(meas.t, z); },
                       [&](const pointPosMeas& z) { handlePointPos(meas.t, z); },
                       [&](const fixAndHoldMeas& z) { handleFixAndHold(meas.t, z); },
                       [&](const gpsObsMeas& z) { handleGpsObsMeas(meas.t, z); },
                       [&](const galObsMeas& z) { handleGalObsMeas(meas.t, z); },
                       [&](const gloObsMeas& z) { handleGloObsMeas(meas.t, z); },
                   },
                   meas.z);
    }

    const Meas& rollFilter(const MeasIterator& start, const MeasIterator& end)
    {
        MeasIterator it = start;
        while (it != end)
        {
            const Meas& meas = *it;
            handleMeas(meas);
            ++it;
        }
        return *(--it);
    }

    const Snapshot& snap() const { return states_.back(); }
    Snapshot& snap() { return states_.back(); }

    const UTCTime& t() const { return states_.back().x.t; }
    UTCTime& t() { return states_.back().x.t; }

    const State& x() const { return states_.back().x; }
    State& x() { return states_.back().x; }

    const Input& u() const { return states_.back().u; }
    Input& u() { return states_.back().u; }

    const dxMat& cov() const { return states_.back().cov; }
    dxMat& cov() { return states_.back().cov; }

    std::multiset<Meas> measurements_;

    DiagMat6 point_pos_cov_;
    DiagMat2 gps_obs_cov_;
    DiagMat2 gal_obs_cov_;
    DiagMat2 glo_obs_cov_;
    DiagMat30 fix_and_hold_cov_;
    DiagMat6 imu_cov_;
    Eigen::DiagonalMatrix<double, 60> process_cov_;

    CircularBuffer<Snapshot> states_;
    int state_idx_ = 0;

    EkfType ekf_;

    Meas last_catchup_meas_;
    static constexpr double MAX_DELAY_AGE_S = 1.0;

    SatelliteManager sat_manager_;

    logging::Logger* log_ = nullptr;

    void ephCb(const ephemeris::GPSEphemeris& eph)
    {
        const int cache_id = cacheId(GnssID::GPS, eph.sat);
        if (sat_cache_.find(cache_id) == sat_cache_.end())
        {
            sat_cache_.insert({cache_id, satellite::SatelliteCache()});
        }
        sat_manager_.ephCb(eph);
    }
    void ephCb(const ephemeris::GlonassEphemeris& eph)
    {
        const int cache_id = cacheId(GnssID::Glonass, eph.sat);
        if (sat_cache_.find(cache_id) == sat_cache_.end())
        {
            sat_cache_.insert({cache_id, satellite::SatelliteCache()});
        }
        sat_manager_.ephCb(eph);
    }
    void ephCb(const ephemeris::GalileoEphemeris& eph)
    {
        const int cache_id = cacheId(GnssID::Galileo, eph.sat);
        if (sat_cache_.find(cache_id) == sat_cache_.end())
        {
            sat_cache_.insert({cache_id, satellite::SatelliteCache()});
        }
        sat_manager_.ephCb(eph);
    }

    FRIEND_TEST(RtkEkfEstimator, OrderMeasurements);
};

using RtkEkfEstimator = EkfEstimator<RtkEkf>;

}  // namespace ekf
}  // namespace mc
