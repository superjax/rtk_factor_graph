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
                           const gpsObsMeas::Covariance& gps_obs_cov,
                           const galObsMeas::Covariance& gal_obs_cov,
                           const gloObsMeas::Covariance& glo_obs_cov,
                           const fixAndHoldMeas::Covariance& fix_and_hold_cov,
                           const InputCovariance& imu_cov,
                           const ProcessCovariance& process_cov,
                           logging::Logger* log)
{
    point_pos_cov_ = point_pos_cov;
    gps_obs_cov_ = gps_obs_cov;
    gal_obs_cov_ = gal_obs_cov;
    glo_obs_cov_ = glo_obs_cov;
    fix_and_hold_cov_ = fix_and_hold_cov;
    imu_cov_ = imu_cov;
    process_cov_ = process_cov;

    log_ = log;

    if (log_)
    {
        log_->initStream<int, gpsObsMeas::Residual, gpsObsMeas::ZType, gpsObsMeas::ZType>(
            logging::GPS_OBS_RESIDUAL, {"sat_id", "res", "z", "zhat"});
        log_->initStream<int, galObsMeas::Residual, galObsMeas::ZType, galObsMeas::ZType>(
            logging::GAL_OBS_RESIDUAL, {"sat_id", "res", "z", "zhat"});
        log_->initStream<int, gloObsMeas::Residual, gloObsMeas::ZType, gloObsMeas::ZType>(
            logging::GLO_OBS_RESIDUAL, {"sat_id", "res", "z", "zhat"});
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

void RtkEkfEstimator::handleGpsObsMeas(const UTCTime& meas_time, const gpsObsMeas& z)
{
    if (meas_time > t().quantized())
    {
        handleImu(meas_time, u());
    }
    const auto& cache = getCache(meas_time, GnssID::GPS, z.sat_id);
    const auto res = update<gpsObsMeas>(make_out(snap()), z.z, gps_obs_cov_, u(), cache);
    if (res.ok() && log_)
    {
        const gpsObsMeas::ZType zhat = z.z - res.res();
        log_->log(logging::GPS_OBS_RESIDUAL, meas_time, z.sat_id, res.res(), z.z, zhat);
    }
}

void RtkEkfEstimator::handleGalObsMeas(const UTCTime& meas_time, const galObsMeas& z)
{
    if (meas_time > t().quantized())
    {
        handleImu(meas_time, u());
    }
    const auto& cache = getCache(meas_time, GnssID::Galileo, z.sat_id);
    const auto res = update<galObsMeas>(make_out(snap()), z.z, gps_obs_cov_, u(), cache);
    if (res.ok() && log_)
    {
        const galObsMeas::ZType zhat = z.z - res.res();
        log_->log(logging::GAL_OBS_RESIDUAL, meas_time, z.sat_id, res.res(), z.z, zhat);
    }
}

void RtkEkfEstimator::handleGloObsMeas(const UTCTime& meas_time, const gloObsMeas& z)
{
    if (meas_time > t().quantized())
    {
        handleImu(meas_time, u());
    }
    const auto& cache = getCache(meas_time, GnssID::Glonass, z.sat_id);
    const auto res = update<gloObsMeas>(make_out(snap()), z.z, gps_obs_cov_, u(), cache);
    if (res.ok() && log_)
    {
        const gloObsMeas::ZType zhat = z.z - res.res();
        log_->log(logging::GLO_OBS_RESIDUAL, meas_time, z.sat_id, res.res(), z.z, zhat);
    }
}

void RtkEkfEstimator::handleMeas(const Meas& meas)
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
