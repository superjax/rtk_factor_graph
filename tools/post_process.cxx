#include "common/defs.h"
#include "common/logging/log_key.h"
#include "common/logging/log_reader.h"
#include "common/logging/log_writer.h"
#include "common/math/two_jet.h"
#include "common/measurements/gnss_observation.h"
#include "common/measurements/imu.h"
#include "core/ekf/rtk_ekf_estimator.h"
#include "third_party/argparse/argparse.hpp"
#include "utils/config.h"
#include "utils/wgs84.h"

using namespace mc;
using namespace ekf;
using namespace third_party::argparse;

ErrorState getStateCovariance(const std::string& key, utils::Config& cfg)
{
    ErrorState stdev;
    // Vec3 trans, rot, vel, acc_bias, gyro_bias, p_b2g, rot_I2e, trans_I2e;
    // Vec2 gps_clk, gal_clk, glo_clk;
    // double sd;
    cfg.get(key + "/rotation", make_out(stdev.pose.head<3>()), true);
    cfg.get(key + "/translation", make_out(stdev.pose.tail<3>()), true);
    cfg.get(key + "/vel", make_out(stdev.vel), true);
    cfg.get(key + "/acc_bias", make_out(stdev.acc_bias), true);
    cfg.get(key + "/gyro_bias", make_out(stdev.gyro_bias), true);
    cfg.get(key + "/gps_clk", make_out(stdev.gps_clk), true);
    cfg.get(key + "/gal_clk", make_out(stdev.gal_clk), true);
    cfg.get(key + "/glo_clk", make_out(stdev.glo_clk), true);
    cfg.get(key + "/p_b2g", make_out(stdev.p_b2g), true);
    cfg.get(key + "/T_I2e", make_out(stdev.T_I2e), true);
    double sd_stdev;
    cfg.get(key + "/sd", make_out(sd_stdev), true);
    stdev.sd.setConstant(sd_stdev);

    ErrorState cov_diag = stdev.cwiseProduct(stdev);
    return cov_diag;
}

Input getInput(utils::Config& cfg)
{
    Input u;
    u.accel << 0, 0, -9.80665;
    u.gyro.setZero();
    return u;
}

std::pair<State, Input> getState(utils::Config& cfg, logging::LogReader& log)
{
    State x;
    Input u;

    Vec3 translation = Vec3::Zero();
    math::Quat<double> q = math::Quat<double>::Identity();
    Vec3 lla_deg = Vec3::Zero();
    double heading_deg = 0.0;

    bool use_init_sim_state = false;
    cfg.get("use_init_sim_state", make_out(use_init_sim_state), true);

    if (use_init_sim_state)
    {
        const auto& entry = log.getNext<math::TwoJet<double>>(logging::TRUTH_POSE);
        x.t = entry.t;
        const math::TwoJet<double>& data = std::get<0>(entry.data);
        x.pose = data.x;
        x.vel = data.dx.linear();
        u.accel = data.d2x.linear() - Vec3(0, 0, 9.80665);
        u.gyro = data.dx.angular();
    }
    else
    {
        x.t = log.startTime();
        cfg.get("translation0", make_out(translation), true);
        cfg.get("rotation0", make_out(q.arr_), true);
        cfg.get("vel0", make_out(x.vel), true);
        cfg.get("p_b2g0", make_out(x.p_b2g), true);
        cfg.get("lla0_deg", make_out(lla_deg), true);
        cfg.get("heading0_deg", make_out(heading_deg), true);
    }

    x.pose = math::DQuat<double>(q, translation);
    x.gps_clk.setZero();
    x.gal_clk.setZero();
    x.glo_clk.setZero();
    x.acc_bias.setZero();
    x.gyro_bias.setZero();

    const Vec3 lla_rad(deg2Rad(lla_deg(0)), deg2Rad(lla_deg(1)), lla_deg(2));
    const double heading_rad = deg2Rad(heading_deg);
    math::Quatd q_hdg = math::Quatd::from_euler(0, 0, heading_rad);

    x.T_I2e = utils::WGS84::dq_ecef2ned(utils::WGS84::lla2ecef(lla_rad)).inverse();
    x.T_I2e.real() = q_hdg * x.T_I2e.real();

    return std::make_pair(x, u);
}

template <typename T>
T getMeasurementCovariance(const std::string& key, utils::Config& cfg)
{
    T vec;
    cfg.get(key, make_out(vec), true);
    return vec.cwiseProduct(vec);
}

int main(int argc, char** argv)
{
    std::string config_file = "";
    std::string log_dir = "";

    ArgumentParser parser("Post-process a data log");
    parser.add_argument("--config_file")
        .help("config file to read (yaml)")
        .action([&](const std::string& value) { config_file = value; })
        .required();
    parser.add_argument("--log")
        .help("log_to_read")
        .action([&](const std::string& value) { log_dir = value; })
        .required();
    parser.parse_args(argc, argv);

    {
        std::ifstream f(config_file);
        std::cout << f.rdbuf() << std::endl;
    }
    utils::Config cfg(config_file);
    logging::LogReader log_reader(log_dir);
    ekf::RtkEkfEstimator ekf;

    const auto x_and_u = getState(cfg, log_reader);
    State x0 = x_and_u.first;
    Input u = x_and_u.second;
    dxVec P0 = getStateCovariance("P0_stdev", cfg);
    dxVec process_cov = getStateCovariance("process_stdev", cfg);

    Vec6 point_pos_cov = getMeasurementCovariance<Vec6>("point_pos_stdev", cfg);
    Vec2 gps_obs_cov = getMeasurementCovariance<Vec2>("gps_obs_stdev", cfg);
    Vec2 gal_obs_cov = getMeasurementCovariance<Vec2>("gal_obs_stdev", cfg);
    Vec2 glo_obs_cov = getMeasurementCovariance<Vec2>("glo_obs_stdev", cfg);
    Input imu_cov;
    imu_cov.accel = getMeasurementCovariance<Vec3>("imu_stdev/accel", cfg);
    imu_cov.gyro = getMeasurementCovariance<Vec3>("imu_stdev/gyro", cfg);
    double fix_and_hold_cov;
    cfg.get("fix_and_hold_stdev", make_out(fix_and_hold_cov), true);

    ekf.init(x0.t, x0, P0, point_pos_cov, gps_obs_cov, gal_obs_cov, glo_obs_cov, fix_and_hold_cov,
             imu_cov, process_cov);

    ekf.u() = u;

    logging::Logger log_writer(x0.t, "../logs/post_process");
    log_writer.initStream<math::DQuat<double>, Vec3, Vec3, Vec3, dxMat>(
        10, {"pose", "vel", "omg", "acc", "cov"});
    log_writer.amends(log_reader.logPath());

    log_reader.setMaintenanceCallback(0.01, [&](const UTCTime& t) {
        if ((t - x0.t).toSec() > 45.0)
        {
            return Error::create("exit early");
        }
        log_writer.log(10, ekf.t(), ekf.x().pose, ekf.x().vel, ekf.u().gyro, ekf.u().accel,
                       ekf.cov());

        return Error::none();
    });

    log_reader.setCallback(logging::LogKey::IMU_SAMPLE,
                           [&](const UTCTime& t, int key, auto& reader) {
                               meas::ImuSample imu;
                               reader.get(imu);
                               ImuMeas meas;
                               meas.z << imu.accel, imu.gyro;
                               ekf.addMeasurement(t, meas);
                           });

    log_reader.setCallback(logging::LogKey::GNSS_OBS, [&](const UTCTime& t, int key, auto& reader) {
        meas::GnssObservation obs;
        reader.get(obs);
        switch (obs.gnss_id)
        {
        case GnssID::GPS: {
            gpsObsMeas meas;
            meas.z << obs.pseudorange, obs.doppler;
            meas.sat_id = obs.sat_num;
            ekf.addMeasurement(t, meas);
            info("gps_obs, {}", mc::fmt(t));
            break;
        }
        case GnssID::Galileo: {
            galObsMeas meas;
            meas.z << obs.pseudorange, obs.doppler;
            meas.sat_id = obs.sat_num;
            info("gal_obs, {}", mc::fmt(t));
            ekf.addMeasurement(t, meas);
            break;
        }
        case GnssID::Glonass: {
            gloObsMeas meas;
            meas.z << obs.pseudorange, obs.doppler;
            meas.sat_id = obs.sat_num;
            info("glo_obs, {}", mc::fmt(t));
            ekf.addMeasurement(t, meas);
            break;
        }
        }
    });

    log_reader.setCallback(logging::LogKey::GPS_EPH, [&](const UTCTime& t, int key, auto& reader) {
        ephemeris::GPSEphemeris eph;
        reader.get(eph);
        const double dt = (t - eph.toe).toSec();
        if (std::abs(dt) > 7200)
        {
            info("Stale ephemeris: dt = {}.  eph.toe = {}, t = {}", mc::fmt(dt, eph.toe, t));
            return;
        }
        ekf.ephCb(eph);
        info("gps_eph, {}", mc::fmt(t));
    });

    log_reader.setCallback(logging::LogKey::GAL_EPH, [&](const UTCTime& t, int key, auto& reader) {
        ephemeris::GalileoEphemeris eph;
        reader.get(eph);
        const double dt = (t - eph.toe).toSec();
        if (std::abs(dt) > 7200)
        {
            info("Stale ephemeris: dt = {}.  eph.toe = {}, t = {}", mc::fmt(dt, eph.toe, t));
            return;
        }
        ekf.ephCb(eph);
        info("gal_eph, {}", mc::fmt(t));
    });

    log_reader.setCallback(logging::LogKey::GLO_EPH, [&](const UTCTime& t, int key, auto& reader) {
        ephemeris::GlonassEphemeris eph;
        reader.get(eph);
        const double dt = (t - eph.toe).toSec();
        if (std::abs(dt) > 7200)
        {
            info("Stale ephemeris: dt = {}.  eph.toe = {}, t = {}", mc::fmt(dt, eph.toe, t));
            return;
        }
        ekf.ephCb(eph);
        info("glo_eph, {}", mc::fmt(t));
    });

    log_reader.read();
}
