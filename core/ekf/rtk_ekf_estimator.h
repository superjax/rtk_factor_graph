#pragma once

#include <gtest/gtest_prod.h>

#include <set>
#include <variant>

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
        size_t idx = 0;  // used to disambiguate identical timestamps

        inline bool operator<(const Meas& other) const
        {
            if (t == other.t)
            {
                return idx < other.idx;
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
              const Vec30& fix_and_hold_cov,
              const Vec6& imu_cov,
              const Vec<60>& process_cov)
    {
        point_pos_cov_ = point_pos_cov.asDiagonal();
        gps_obs_cov_ = gps_obs_cov.asDiagonal();
        gal_obs_cov_ = gal_obs_cov.asDiagonal();
        glo_obs_cov_ = glo_obs_cov.asDiagonal();
        fix_and_hold_cov_ = fix_and_hold_cov.asDiagonal();
        imu_cov_ = imu_cov.asDiagonal();
        process_cov_ = process_cov.asDiagonal();

        const auto init_ekf = [&](EkfType& ekf) {
            ekf.x() = x0;
            ekf.x().t = t0;
            ekf.P() = P0.asDiagonal();
        };

        init_ekf(propagated_ekf_);
        init_ekf(delayed_ekf_);
        init_ekf(catchup_ekf_);
    }

    template <typename T>
    Error addMeasurement(const UTCTime& t, const T& z)
    {
        if (t < delayed_ekf_.t())
        {
            return Error::create("cannot handle stale measurement");
        }

        static size_t meas_idx = 0;
        auto it = measurements_.insert(Meas{t, z, ++meas_idx});

        if (t < propagated_ekf_.t())
        {
            // Out-of-order measurement.  We need to rewind to apply it.
            MeasIterator catchup_begin;
            if (t < catchup_ekf_.t())
            {
                // This is older than our catchup filter, go all the way back to delayed
                catchup_ekf_ = delayed_ekf_;
                catchup_begin = measurements_.begin();
            }
            else
            {
                // not older than catchup
                catchup_begin = measurements_.find(last_catchup_meas_);
                if (catchup_begin == measurements_.end())
                {
                    // This happens if `last_catchup_meas_` is uninitialized
                    check(measurements_.begin()->t >= catchup_ekf_.t());
                    catchup_begin = measurements_.begin();
                }
                else
                {
                    catchup_begin++;
                }
            }
            last_catchup_meas_ = rollFilter(catchup_ekf_, catchup_begin, ++it);
            propagated_ekf_ = catchup_ekf_;
        }

        rollFilter(propagated_ekf_, it, measurements_.end());

        drop_old_measurements();

        return Error::none();
    }

    void drop_old_measurements()
    {
        check(!measurements_.empty(), "cannot process empty buffer");
        auto first = measurements_.begin();
        auto last = measurements_.crbegin();

        while ((last->t - first->t).toSec() > MAX_DELAY_AGE_S)
        {
            handleMeas(delayed_ekf_, *first);
            measurements_.erase(first);
            first = measurements_.begin();
        }

        if (delayed_ekf_.t() > catchup_ekf_.t())
        {
            catchup_ekf_ = delayed_ekf_;
            last_catchup_meas_ = Meas{};
        }
    }

    mutable std::unordered_map<int, satellite::SatelliteCache> sat_cache_;
    static constexpr int cacheId(int gnss_id, int sat_num) { return (gnss_id << 8) | sat_num; }

    const satellite::SatelliteCache& getCache(const UTCTime& t,
                                              const Vec3 pe2g,
                                              int gnss_id,
                                              int sat_id) const
    {
        auto& cache = sat_cache_.at(cacheId(gnss_id, sat_id));
        const satellite::SatelliteBase* sat;
        const Error res = sat_manager_.getSat(gnss_id, sat_id, make_out(sat));
        check(res.ok(), "unable to retrieve satellite");

        cache.update(t, pe2g, *sat);
        return cache;
    }

    void handleImu(EkfType& ekf, const UTCTime& t, const ImuMeas& z) const
    {
        if (t > ekf.t())
        {
            ekf.predict(t, z.z, process_cov_, imu_cov_);
        }
    }

    void handlePointPos(EkfType& ekf, const UTCTime& t, const pointPosMeas& z) const
    {
        if (t > ekf.t())
        {
            ekf.predict(t, ekf.u(), process_cov_, imu_cov_);
        }
        ekf.template update<pointPosMeas>(z.z, point_pos_cov_);
    }

    void handleFixAndHold(EkfType& ekf, const UTCTime& t, const fixAndHoldMeas& z) const
    {
        if (t > ekf.t())
        {
            ekf.predict(t, ekf.u(), process_cov_, imu_cov_);
        }
        ekf.template update<fixAndHoldMeas>(z.z, fix_and_hold_cov_);
    }

    void handleGpsObsMeas(EkfType& ekf, const UTCTime& t, const gpsObsMeas& z) const
    {
        if (t > ekf.t())
        {
            ekf.predict(t, ekf.u(), process_cov_, imu_cov_);
        }
        ekf.template update<gpsObsMeas>(z.z, gps_obs_cov_,
                                        getCache(t, ekf.p_e_g2e(), GnssID::GPS, z.sat_id));
    }

    void handleGalObsMeas(EkfType& ekf, const UTCTime& t, const galObsMeas& z) const
    {
        if (t > ekf.t())
        {
            ekf.predict(t, ekf.u(), process_cov_, imu_cov_);
        }
        ekf.template update<galObsMeas>(z.z, gal_obs_cov_,
                                        getCache(t, ekf.p_e_g2e(), GnssID::Galileo, z.sat_id));
    }

    void handleGloObsMeas(EkfType& ekf, const UTCTime& t, const gloObsMeas& z) const
    {
        if (t > ekf.t())
        {
            ekf.predict(t, ekf.u(), process_cov_, imu_cov_);
        }
        ekf.template update<gloObsMeas>(z.z, glo_obs_cov_,
                                        getCache(t, ekf.p_e_g2e(), GnssID::Glonass, z.sat_id));
    }

    void handleMeas(EkfType& ekf, const Meas& meas) const
    {
        std::visit(detail::overloaded{
                       [&](const ImuMeas& z) { handleImu(ekf, meas.t, z); },
                       [&](const pointPosMeas& z) { handlePointPos(ekf, meas.t, z); },
                       [&](const fixAndHoldMeas& z) { handleFixAndHold(ekf, meas.t, z); },
                       [&](const gpsObsMeas& z) { handleGpsObsMeas(ekf, meas.t, z); },
                       [&](const galObsMeas& z) { handleGalObsMeas(ekf, meas.t, z); },
                       [&](const gloObsMeas& z) { handleGloObsMeas(ekf, meas.t, z); },
                   },
                   meas.z);
    }

    const Meas& rollFilter(EkfType& ekf, const MeasIterator& start, const MeasIterator& end) const
    {
        MeasIterator it = start;
        while (it != end)
        {
            const Meas& meas = *it;
            handleMeas(ekf, meas);
            ++it;
        }
        return *(--it);
    }

    std::multiset<Meas> measurements_;

    DiagMat6 point_pos_cov_;
    DiagMat2 gps_obs_cov_;
    DiagMat2 gal_obs_cov_;
    DiagMat2 glo_obs_cov_;
    DiagMat30 fix_and_hold_cov_;
    DiagMat6 imu_cov_;
    Eigen::DiagonalMatrix<double, 60> process_cov_;

    EkfType propagated_ekf_;
    EkfType delayed_ekf_;
    EkfType catchup_ekf_;

    Meas last_catchup_meas_;
    static constexpr double MAX_DELAY_AGE_S = 1.0;

    SatelliteManager sat_manager_;

    FRIEND_TEST(RtkEkfEstimator, OrderMeasurements);
};

}  // namespace ekf
}  // namespace mc
