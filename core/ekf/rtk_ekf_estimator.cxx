#include "core/ekf/rtk_ekf_estimator.h"

namespace mc {
namespace ekf {

static constexpr int cacheId(int gnss_id, int sat_num)
{
    return (gnss_id << 8) | sat_num;
}

ImuMeas ImuMeas::Zero()
{
    ImuMeas out;
    out.z.accel.setZero();
    out.z.gyro.setZero();
    return out;
}

void RtkEkfEstimator::init(const UTCTime& t0,
                           const State& x0,
                           const Covariance& P0,
                           const pointPosMeas::Covariance& point_pos_cov,
                           const obsMeas::Covariance& obs_cov,
                           const fixAndHoldMeas::Covariance& fix_and_hold_cov,
                           const InputCovariance& imu_cov,
                           const ProcessCovariance& process_cov,
                           logging::Logger* log)
{
    point_pos_cov_ = point_pos_cov;
    obs_cov_ = obs_cov;
    fix_and_hold_cov_ = fix_and_hold_cov;
    imu_cov_ = imu_cov;
    process_cov_ = process_cov;

    log_ = log;

    if (log_)
    {
        log_->initStream<int, Vec2, Vec2, Vec2>(logging::GPS_OBS_RESIDUAL,
                                                {"sat_id", "res", "z", "zhat"});
        log_->initStream<pointPosMeas::Residual, pointPosMeas::ZType, pointPosMeas::ZType>(
            logging::POINT_POS_RESIDUAL, {"res", "z", "zhat"});
        log_->initStream<fixAndHoldMeas::Residual, fixAndHoldMeas::Residual,
                         fixAndHoldMeas::Residual>(logging::FIX_AND_HOLD_RESIDUAL,
                                                   {"res", "z", "zhat"});
    }

    Input u0;
    u0.accel = -GRAVITY;
    u0.gyro.setZero();

    states_.emplace_back();
    x() = x0;
    t() = t0;
    cov() = P0;
    u() = u0;
}

void RtkEkfEstimator::dropOldMeasurements()
{
    check(!measurements_.empty(), "cannot process empty buffer");

    const UTCTime t_start = measurements_.crbegin()->t - MAX_DELAY_AGE_S;
    measurements_.erase(measurements_.cbegin(), measurements_.lower_bound(Meas{t_start, {}}));
    while (states_.front().x.t < t_start)
    {
        states_.pop_front();
    }
}

bool RtkEkfEstimator::hasCache(int gnss_id, int sat_num) const
{
    return sat_cache_.find(cacheId(gnss_id, sat_num)) != sat_cache_.end();
}

const satellite::SatelliteCache& RtkEkfEstimator::getCache(const UTCTime& t,
                                                           int gnss_id,
                                                           int sat_id) const
{
    auto& cache = sat_cache_.at(cacheId(gnss_id, sat_id));
    const satellite::SatelliteBase* sat;
    const Error res = sat_manager_.getSat(gnss_id, sat_id, make_out(sat));
    check(res.ok(), "unable to retrieve satellite");

    cache.update(t, p_ecef(), *sat);
    return cache;
}

void RtkEkfEstimator::handleImu(const UTCTime& meas_time, const ImuMeas& z)
{
    handleImu(meas_time, z.z);
}

void RtkEkfEstimator::handleImu(const UTCTime& meas_time, const Input& z)
{
    if (meas_time > t())
    {
        Snapshot& snap = states_.back();
        states_.emplace_back();
        Snapshot& new_snap = states_.back();
        predict(snap, meas_time, z, process_cov_, imu_cov_, make_out(new_snap));
    }
}

void RtkEkfEstimator::handlePointPos(const UTCTime& meas_time, const pointPosMeas& z)
{
    if (meas_time > t().quantized())
    {
        handleImu(meas_time, u());
    }
    const auto res = update<pointPosMeas>(make_out(snap()), z.z, point_pos_cov_, u());
    if (res.ok() && log_)
    {
        const pointPosMeas::ZType zhat = z.z - res.res();
        log_->log(logging::POINT_POS_RESIDUAL, meas_time, res.res(), z.z, zhat);
    }
}

void RtkEkfEstimator::handleFixAndHold(const UTCTime& meas_time, const fixAndHoldMeas& z)
{
    if (meas_time > t().quantized())
    {
        handleImu(meas_time, u());
    }
    const auto res = update<fixAndHoldMeas>(make_out(snap()), z.z, fix_and_hold_cov_);
    if (res.ok() && log_)
    {
        // Logging needs to be a fixed size
        Vec<fixAndHoldMeas::MAX_SIZE> zhat;
        Vec<fixAndHoldMeas::MAX_SIZE> z_full;
        for (size_t i = 0; i < z.z.size(); ++i)
        {
            zhat[i] = z.z[i](0) - res.res()[i];
            z_full[i] = z.z[i](0);
        }
        for (size_t i = z.z.size(); i < fixAndHoldMeas::MAX_SIZE; ++i)
        {
            zhat[i] = NAN;
            z_full[i] = NAN;
        }
        log_->log(logging::FIX_AND_HOLD_RESIDUAL, meas_time, res.res(), z_full, zhat);
    }
}

void RtkEkfEstimator::handleObsMeas(const UTCTime& meas_time,
                                    const std::vector<meas::GnssObservation>& obs_bundle)
{
    if (meas_time > t().quantized())
    {
        handleImu(meas_time, u());
    }
    obsMeas::ZType z;
    z.reserve(obs_bundle.size());
    for (const auto& o : obs_bundle)
    {
        check(o.t == meas_time, "all measurement timestamps in a GNSS bundle must match");
        z.push_back({Vec2(o.pseudorange, o.doppler), getCache(o.t, o.gnss_id, o.sat_num)});
    }

    const auto res = update<obsMeas>(make_out(snap()), z, obs_cov_, u());
    if (res.ok() && log_)
    {
        for (size_t i = 0; i < z.size(); ++i)
        {
            const Vec2& z_i = z[i].z;
            const Vec2 residual_i = res.res().segment<2>(i * 2);
            const Vec2 zhat = z_i - residual_i;
            const int sat_id = obs_bundle[i].sat_num;
            log_->log(logging::GPS_OBS_RESIDUAL, meas_time, sat_id, residual_i, z_i, zhat);
        }
    }
}

void RtkEkfEstimator::handleMeas(const Meas& meas)
{
    std::visit(detail::overloaded{
                   [&](const ImuMeas& z) { handleImu(meas.t, z); },
                   [&](const pointPosMeas& z) { handlePointPos(meas.t, z); },
                   [&](const fixAndHoldMeas& z) { handleFixAndHold(meas.t, z); },
                   [&](const std::vector<meas::GnssObservation>& z) { handleObsMeas(meas.t, z); },
               },
               meas.z);
}

const RtkEkfEstimator::Meas& RtkEkfEstimator::rollFilter(const MeasIterator& start,
                                                         const MeasIterator& end)
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

Vec3 RtkEkfEstimator::p_ecef() const
{
    const State& x = snap().x;
    return x.T_I2e.transformp(x.pose.transforma(x.p_b2g));
}

}  // namespace ekf
}  // namespace mc
